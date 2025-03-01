#define B_LDR_IMPLEMENTATION
#include "./b_ldr.hpp"

std::string TARGET = "main";
std::string TARGET_FOLDER = "./build";

void add_flags(bld::Command &cmd) { cmd.add_parts("-O3", "-Wall", "-Wextra", "-Wno-missing-field-initializers"); }

void add_include_flags(bld::Command &cmd) { cmd.add_parts("-I./"); }

void add_linker_flags(bld::Command &cmd)
{
  cmd.add_parts("./raylib/lib/libraylib.a",
                "-L./ "
                "-lm");
}

std::string get_file_name(const std::string &file)
{
  std::filesystem::path p(file);
  return p.filename().string();
}

void add_build_source(bld::Command &cmd)
{
  std::string FOLDER = "./src/";
  auto files = bld::fs::list_files_in_dir(FOLDER);
  for (auto &file : files)
    if (bld::fs::get_extension(file) == ".cpp") cmd.add_parts(file);
}

void handle_args(const std::vector<std::string> &args)
{
  if (args.size() == 1 && (args[0] == "help" || args[0] == "-help" || args[0] == "-h"))
  {
    std::cout << "Usage: bld [command]\n\n"
                 "Commands:\n"
                 "    run    - Run the main executable\n"
                 "    clean  - Clean the build folder\n"
                 "    help   - Show this message\n";
    exit(0);
  }
  else if (args.size() == 1 && args[0] == "run")
  {
    if (bld::execute({TARGET_FOLDER + "/" + TARGET}) < 1) exit(1);
    ;
    exit(0);
  }
  else if (args.size() == 1 && args[0] == "clean")
  {
    if (!bld::fs::remove_dir(TARGET_FOLDER)) exit(1);
    exit(0);
  }
}

int main(int argc, char **argv)
{
  BLD_REBUILD_YOURSELF_ONCHANGE();
  std::vector<std::string> args{};

  if (!bld::args_to_vec(argc, argv, args)) return 1;
  handle_args(args);

  bld::Command cmd{};

  bld::fs::create_dir_if_not_exists(TARGET_FOLDER);

  cmd.add_parts("g++", "-o", TARGET_FOLDER + "/" + TARGET);
  add_flags(cmd);
  cmd.add_parts("./main.cpp");
  add_include_flags(cmd);
  add_linker_flags(cmd);

  add_build_source(cmd);
  if (bld::execute(cmd) < 1) return 1;

  return 0;
}
