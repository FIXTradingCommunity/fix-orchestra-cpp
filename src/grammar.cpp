#include <grammar.h>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/support.hpp>

namespace score {
namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;

template <typename Iterator>
struct Grammar : qi::grammar<Iterator, ast::Statement(), ascii::space_type> {
  Grammar() : Grammar::base_type(statementRule) {
    // TODO
  }

  qi::rule<Iterator, ast::Statement(), ascii::space_type> statementRule;
};

bool parse(ast::Statement &out, const std::string &in) {
  using boost::spirit::ascii::space;

  score::Grammar<std::string::const_iterator> grammar;

  const auto &iter = in.cbegin();
  const auto &end = in.cend();
  bool parsed = phrase_parse(iter, end, grammar, space, out);

  return parsed;
}
}
