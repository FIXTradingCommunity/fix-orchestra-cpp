#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/fusion/adapted/struct/adapt_struct.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/fusion/adapted/std_tuple.hpp>
#include <boost/fusion/adapted/std_pair.hpp>

#include <boost/spirit/include/qi.hpp>

#include <boost/optional/optional.hpp>
#include <boost/optional/optional_io.hpp>

#include <boost/variant/variant.hpp>
#include <boost/variant/recursive_variant.hpp>

namespace client
{
  struct var;

  using expr = boost::variant<
                   unsigned int // UINT
                 , int          // INT
                 , char         // CHAR
                 , std::string       // STRING
                 , boost::recursive_wrapper<var>>;
  struct assignment {
      boost::recursive_wrapper<var>  lhs;
      expr                           rhs;
  };
  using score_expression = boost::variant<assignment, expr>;

  using qual_inner = boost::variant< int, std::pair<std::string, expr>>;
  struct qual {
      std::string id;
      qual_inner  inner;
  };
  struct var {
      boost::optional<std::string> scope;
      std::vector<qual>            qualifiedVariable;
  };
  std::ostream& operator<<(std::ostream& os, const qual& q)
  {
      os << q.id; // TODO: inner...
      return os;
  }
  std::ostream& operator<<(std::ostream& os, const var& v)
  {
      os << "scope: " << v.scope << ", qualifiedVariable:";
      for (const auto& q : v.qualifiedVariable)
        os << " " << q;
      return os;
  }
  std::ostream& operator<<(std::ostream& os, const assignment& a)
  {
      os << a.lhs.get() << "=" << a.rhs;
      return os;
  }
}

BOOST_FUSION_ADAPT_STRUCT(client::assignment,
                          (boost::recursive_wrapper<client::var>, lhs),
                          (client::expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(client::qual,
                          (std::string, id),
                          (client::qual_inner, inner))
BOOST_FUSION_ADAPT_STRUCT(client::var,
                          (boost::optional<std::string>, scope),
                          (std::vector<client::qual>, qualifiedVariable))

namespace client
{
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;

    template <typename Iterator>
    struct score_expression_grammar
      : qi::grammar<Iterator, score_expression(), ascii::space_type>
    {
        score_expression_grammar() : score_expression_grammar::base_type(score_expression_rule)
        {
          using qi::int_;
          using qi::uint_;
          using qi::double_;
          using qi::char_;
          using qi::digit;
          using qi::alpha;
          using qi::lit;
          using qi::lexeme;
          using qi::string;

          score_expression_rule = assignment_rule
                                | expr_rule;

          expr_rule = uint_
                    | int_
                    | char_rule
                    | string_rule
                    | var_rule;

          assignment_rule = var_rule >> lit('=') >> expr_rule;

          index_rule = lit('[') >> uint_ >> lit(']');
          pred_rule  = lit('[') >> id_rule >> lit("==") >> expr_rule >> lit(']');
          scope_rule = -(string("$") | string("^") | string("in.") | string("out.") | string("this."));
          var_rule   = scope_rule >> qual_rule >> ("." % qual_rule);

          string_rule = lexeme['"' >> +stringchar_rule >> '"'];
          char_rule   = '\'' >> stringchar_rule >> '\'';

          id_rule = alpha >> -(alpha | digit | lit('_'));

          stringchar_rule = ~char_("\\\"")
                          | esc_rule;
          esc_rule = '\\' >> char_("btnfr\\\"'");

          BOOST_SPIRIT_DEBUG_NODES((score_expression_rule)(string_rule)(stringchar_rule)(esc_rule));
        }

        qi::rule<Iterator, score_expression(), ascii::space_type> score_expression_rule;
        qi::rule<Iterator, assignment()> assignment_rule;
        qi::rule<Iterator, expr()> expr_rule;

        qi::rule<Iterator, unsigned int()> index_rule;
        qi::rule<Iterator, std::pair<std::string, expr>()> pred_rule;
        qi::rule<Iterator, qual()> qual_rule;
        qi::rule<Iterator, boost::optional<std::string>()> scope_rule;
        qi::rule<Iterator, var()> var_rule;

        qi::rule<Iterator, std::string()> string_rule;
        qi::rule<Iterator, char()>        char_rule;

        qi::rule<Iterator, std::string()> id_rule;

        qi::rule<Iterator, char()> stringchar_rule;
        qi::rule<Iterator, char()> esc_rule;
    };
}

int main(int argc, char *argv[])
{
    char const* filename;
    if (argc > 1)
    {
        filename = argv[1];
    }
    else
    {
        std::cerr << "Error: No input file provided." << std::endl;
        return 1;
    }

    std::ifstream in(filename, std::ios_base::in);

    if (!in)
    {
        std::cerr << "Error: Could not open input file: "
            << filename << std::endl;
        return 1;
    }

    std::string storage; // We will read the contents here.
    in.unsetf(std::ios::skipws); // No white space skipping!
    std::copy(
        std::istream_iterator<char>(in),
        std::istream_iterator<char>(),
        std::back_inserter(storage));

    typedef client::score_expression_grammar<std::string::const_iterator> score_expression_grammar;
    score_expression_grammar grammar; // Our grammar
    client::score_expression ast; // Our tree

    using boost::spirit::ascii::space;
    std::string::const_iterator iter = storage.begin();
    std::string::const_iterator saved = iter;
    std::string::const_iterator end = storage.end();
    bool r = phrase_parse(iter, end, grammar, space, ast);

    if (r && iter == end)
    {
        std::cout << "-------------------------\n";
        std::cout << "Parsing succeeded\n";
        std::cout << "-------------------------\n";
        std::cout << "the string: " << ast << "\n";
        return 0;
    }
    else
    {
        std::string context(saved, iter);
        std::cout << "-------------------------\n";
        std::cout << "Parsing failed\n";
        std::cout << "stopped at: \": " << context << "...\"\n";
        std::cout << "-------------------------\n";
        return 1;
    }

}
