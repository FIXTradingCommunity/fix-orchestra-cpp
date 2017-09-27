#include <ast.h>
#include <grammar.h>

#include <boost/program_options.hpp>

#include <iostream>
#include <string>
#include <type_traits>

const std::string testsSuccess[] = {"42",
                                    "-12",
                                    "3*4",
                                    "10 mod 2",
                                    "-1*(4123%(1 + 4))",
                                    "!(4/2)",
                                    "-(3*(2/2))",
                                    "4*2-3",
                                    "(2+3)*4",
                                    "2+2+2",
                                    "3==4",
                                    "5 ne 6",
                                    "3+4>22",
                                    "5*(3+2) le 1",
                                    "(2+4) between [-1, 10]",
                                    "10 in {4}",
                                    "!(2 in {3, 4, 5})",
                                    "(6 > 4) and (1 == 1)",
                                    "(3 mod 2 == 0) or 3 in {1, 2, 3}",
                                    "3.25 < 4-0.5",
                                    "'a' > '\\t'",
                                    "\"hello\"==\"world\\n\"",
                                    "$x.y[123].z",
                                    "q[xy_2==(1+2)]"};

const std::string testsFailure[] = {
    "4*", "/2",
    /* "1*+2", */ // interesting case - double_ parses this...so is it wrong?
    "1+*2", "3+", "!between [2,10]"};

std::pair<bool, score::ast::Statement> doParse(const std::string &stmt) {
  score::ast::Statement ast;
  bool parsed = score::parse(ast, stmt);
  return {parsed, ast};
}

void runTests() {
  auto testFn = [](auto &tests, bool shouldParse) {
    for (const auto &t : tests) {
      std::cout << "attempting parse for: " << t << std::endl;

      bool parsed = doParse(t).first;

      if ((shouldParse && parsed) || (!shouldParse && !parsed)) {
        std::cout << "\tOK" << std::endl;
      } else {
        std::cout << "\tERROR "
                  << "(should" << (shouldParse ? " " : " not ")
                  << "have parsed)" << std::endl;
      }
    }
  };
  testFn(testsSuccess, true);
  testFn(testsFailure, false);
}

int main(int argc, char *argv[]) {
  namespace po = boost::program_options;

  try {
    po::options_description scoreDesc("score dsl command-line tool");
    scoreDesc.add_options()("help,h", "this help message")(
        "runTests", "run built-in test suite")("expression",
                                               "score expression to evaluate");

    po::positional_options_description posOptions;
    posOptions.add("expression", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv)
                  .options(scoreDesc)
                  .positional(posOptions)
                  .run(),
              vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << scoreDesc << std::endl;
      return EXIT_SUCCESS;
    }

    const auto &rtCount = vm.count("runTests");
    const auto &eCount = vm.count("expression");
    if (rtCount && eCount) {
      std::cerr << "either specify the 'runTests' flag or an 'expression', but "
                   "not both."
                << std::endl;
      return EXIT_FAILURE;
    }

    if (rtCount) {
      runTests();
      return EXIT_SUCCESS;
    }

    if (eCount) {
      const auto &eArg = vm["expression"].as<std::string>();

      const auto &parsePair = doParse(eArg);
      if (!parsePair.first) {
        std::cout << "parse failed" << std::endl;
      } else {
        std::cout << "parse succeeded" << std::endl;
      }
      return EXIT_SUCCESS;
    }

    std::cout << scoreDesc << std::endl;
    return EXIT_SUCCESS;
  } catch (std::exception &e) {
    std::cerr << "exception encountered: " << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "unknown failure" << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
