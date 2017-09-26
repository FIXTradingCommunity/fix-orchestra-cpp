#include <ast.h>
#include <grammar.h>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/support.hpp>

#include <iostream>
#include <string>
#include <type_traits>

int main(int argc, char *argv[]) {
  const std::string tests[] = {"42", "-12", "3*4", "!(4/2)", "4*", "/2"};

  for (const auto &t : tests) {
    std::cout << "attempting parse for: " << t << std::endl;

    score::ast::Statement ast;
    bool parsed = score::parse(ast, t);

    if (parsed) {
      std::cout << "\tOK" << std::endl;
    } else {
      std::cout << "\tERROR" << std::endl;
    }
  }
  return EXIT_SUCCESS;
}
