#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <iostream>

class Multi_threader
{
    std::vector<std::thread> _workers;
    std::queue<std::function<void()>> _task_queue;
    std::mutex _queue_mutex;
    std::condition_variable _cv;
    std::atomic<int> _active_tasks;
    std::mutex _completion_mutex;
    std::condition_variable _completion_cv;
    std::atomic<bool> _stop;

  public:
    explicit Multi_threader(size_t thread_count = std::thread::hardware_concurrency()) : _active_tasks(0), _stop(false)
    {
        _workers.reserve(thread_count);
        for (size_t i = 0; i < thread_count; ++i)
        {
            _workers.emplace_back(
                [this]
                {
                    while (true)
                    {
                        std::function<void()> task;
                        {
                            std::unique_lock<std::mutex> lock(_queue_mutex);
                            _cv.wait(lock, [this] { return _stop || !_task_queue.empty(); });
                            if (_stop ) return;  // Exit thread
                            task = _task_queue.front();
                            _task_queue.pop();
                        }
                        try {
                            task(); // Execute the task
                            _active_tasks.fetch_sub(1, std::memory_order_relaxed);
                        }
                        catch (...)
                        {
                            _active_tasks.fetch_sub(1, std::memory_order_relaxed);
                            std::cerr << "Exception in task" << std::endl;
                            throw;
                        }
                        std::lock_guard<std::mutex> lock(_completion_mutex);
                        _completion_cv.notify_one();
                    }
                });
        }
    }

    ~Multi_threader()
    {
        {
            std::lock_guard<std::mutex> lock(_queue_mutex);
            _stop = true;
        }
        _cv.notify_all();
        for (auto &worker : _workers)
            if (worker.joinable())
                worker.join();
    }

    void add_task(std::function<void()> task)
    {
        {
            std::lock_guard<std::mutex> lock(_queue_mutex);
            _task_queue.emplace(std::move(task));
        }
        _active_tasks.fetch_add(1, std::memory_order_relaxed);
        _cv.notify_one();
    }

    void wait()
    {
        std::unique_lock<std::mutex> lock(_completion_mutex); 
        _completion_cv.wait(lock, [this] { return _active_tasks.load(std::memory_order_relaxed) == 0; });
    }
};
