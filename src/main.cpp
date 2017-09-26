#include <iostream>
#include <string>
#include <type_traits>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/support.hpp>

namespace score { namespace ast {
  struct Statement { };
}
}

namespace score
{
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;

    template <typename Iterator>
    struct Grammar
      : qi::grammar<Iterator, ast::Statement(), ascii::space_type>
    {
      Grammar() : Grammar::base_type(statementRule)
      {
        // TODO
      }

      qi::rule<Iterator, ast::Statement(), ascii::space_type> statementRule;
    };
}


int main(int argc, char *argv[])
{
  using boost::spirit::ascii::space;

  const std::string tests[] = {
    "42"
  };
  using IterT = std::remove_reference<decltype(*tests)>::type::const_iterator;

  score::Grammar<IterT> grammar;
  score::ast::Statement ast;

  for (const auto& t : tests) {
    std::cout << "attempting parse for: " << t << std::endl;

    IterT iter = t.cbegin();
    IterT end  = t.cend();
    bool parsed = phrase_parse(iter, end, grammar, space, ast);

    if (parsed && iter == end) {
      std::cout << "\tOK" << std::endl;
    }
    else {
      std::cout << "\tERROR, parsed context: " << std::string(iter, end) << std::endl;
    }
  }
  return EXIT_SUCCESS;
}
