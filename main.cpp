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
  struct exists;

  struct binRel;
  struct binEq;
  struct binAnd;
  struct binOr;

  struct var;

  struct expr {
    using expr_t = boost::variant<
                      boost::recursive_wrapper<expr>
                   ,  boost::recursive_wrapper<binRel>
                   ,  boost::recursive_wrapper<binEq>
                   ,  boost::recursive_wrapper<binAnd>
                   ,  boost::recursive_wrapper<binOr>
                      // # DATETIME #
                      // # TIME #
                      // # DATE #
                      // # PERIOD #
                   ,  unsigned int // UINT
                   , double       // DECIMAL - TODO: change to cpp_dec_float
                   , int          // INT
                   , char         // CHAR
                   , std::string       // STRING
                   , boost::recursive_wrapper<exists>
                   , boost::recursive_wrapper<var>>;
    expr_t expression;
  };
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
  struct exists {
      var variable;
  };

  struct binRel {
      expr lhs;
      std::string relation;
      expr rhs;
  };
  struct binEq {
      expr lhs;
      bool negated;
      expr rhs;
  };
  struct binAnd {
      expr lhs;
      expr rhs;
  };
  struct binOr {
      expr lhs;
      expr rhs;
  };

  std::ostream& operator<<(std::ostream& os, const expr& e)
  {
      os << e.expression;
      return os;
  }
  std::ostream& operator<<(std::ostream& os, const binRel& r)
  {
      os << r.lhs << " " << r.relation << r.rhs;
      return os;
  }
  std::ostream& operator<<(std::ostream& os, const binEq& be)
  {
      os << be.lhs << " " << (be.negated ? "!=" : "==") << " " << be.rhs;
      return os;
  }
  std::ostream& operator<<(std::ostream& os, const binAnd& ba)
  {
      os << ba.lhs << " && " << ba.rhs;
      return os;
  }
  std::ostream& operator<<(std::ostream& os, const binOr& bo)
  {
      os << bo.lhs << " || " << bo.rhs;
      return os;
  }
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
  std::ostream& operator<<(std::ostream& os, const exists& e)
  {
      os << "exists[" << e.variable << "]";
      return os;
  }
}

BOOST_FUSION_ADAPT_STRUCT(client::assignment,
                          (boost::recursive_wrapper<client::var>, lhs),
                          (client::expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(client::expr,
                          (client::expr::expr_t, expression))

BOOST_FUSION_ADAPT_STRUCT(client::binRel,
                          (client::expr, lhs),
                          (std::string, relation),
                          (client::expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(client::binEq,
                          (client::expr, lhs),
                          (bool, negated),
                          (client::expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(client::binAnd,
                          (client::expr, lhs),
                          (client::expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(client::binOr,
                          (client::expr, lhs),
                          (client::expr, rhs))

BOOST_FUSION_ADAPT_STRUCT(client::qual,
                          (std::string, id),
                          (client::qual_inner, inner))
BOOST_FUSION_ADAPT_STRUCT(client::var,
                          (boost::optional<std::string>, scope),
                          (std::vector<client::qual>, qualifiedVariable))
BOOST_FUSION_ADAPT_STRUCT(client::exists,
                          (client::var, variable))

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

          assignment_rule = var_rule >> lit('=') >> expr_rule;

          expr_rule = parens_rule
                    | rel_rule
                    // | eq_rule
                    | and_rule
                    | or_rule
                    // # DATETIME #
                    // # TIME #
                    // # DATE #
                    // # PERIOD #
                    | uint_
                    | decimal_rule
                    | int_
                    | char_rule
                    | string_rule
                    | exists_rule
                    | var_rule;
          rel_rule    = expr_rule >>
                            ( string("<")
                            | string("<=")
                            | string(">")
                            | string(">=")
                            | string("lt")
                            | string("le")
                            | string("gt")
                            | string("ge"))
                        >> expr_rule;
          // eq_rule  = ???;
          and_rule    = expr_rule >> (lit("and") | lit("&&")) >> expr_rule;
          or_rule     = expr_rule >> (lit("or")  | lit("||")) >> expr_rule;
          parens_rule = lit("(") >> expr_rule >> lit(")");
          exists_rule = lit("exists") >> var_rule;

          index_rule = lit('[') >> uint_ >> lit(']');
          pred_rule  = lit('[') >> id_rule >> lit("==") >> expr_rule >> lit(']');
          scope_rule = -(string("$") | string("^") | string("in.") | string("out.") | string("this."));
          var_rule   = scope_rule >> qual_rule >> ("." % qual_rule);

          // TODO: cpp_dec_float
          decimal_rule = double_;

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

        qi::rule<Iterator, binRel()> rel_rule;
        qi::rule<Iterator, binEq()> eq_rule;
        qi::rule<Iterator, binAnd()> and_rule;
        qi::rule<Iterator, binOr()> or_rule;
        qi::rule<Iterator, expr()> parens_rule;
        qi::rule<Iterator, exists()> exists_rule;

        qi::rule<Iterator, unsigned int()> index_rule;
        qi::rule<Iterator, std::pair<std::string, expr>()> pred_rule;
        qi::rule<Iterator, qual()> qual_rule;
        qi::rule<Iterator, boost::optional<std::string>()> scope_rule;
        qi::rule<Iterator, var()> var_rule;

        // TODO: cpp_dec_float
        qi::rule<Iterator, double()> decimal_rule;

        qi::rule<Iterator, std::string()> string_rule;
        qi::rule<Iterator, char()>        char_rule;

        qi::rule<Iterator, std::string()> id_rule;

        qi::rule<Iterator, char()> stringchar_rule;
        qi::rule<Iterator, char()> esc_rule;
    };
}

int main(int argc, char *argv[])
{
    // `main` based off of the tutorials on the boost website done by
    // Joel de Guzman and Helmut Kaiser, e.g.:
    //     http://www.boost.org/doc/libs/1_60_0/libs/spirit/example/qi/parse_date.cpp
    char const* filename;
    if (argc > 1) {
        filename = argv[1];
    }
    else {
        std::cerr << "no input file" << std::endl;
        return 1;
    }

    std::ifstream in(filename, std::ios_base::in);
    if (!in) {
        std::cerr << "could not open file ("
            << filename << ")" << std::endl;
        return 1;
    }

    std::string storage;
    in.unsetf(std::ios::skipws);
    std::copy(
        std::istream_iterator<char>(in),
        std::istream_iterator<char>(),
        std::back_inserter(storage));

    typedef client::score_expression_grammar<std::string::const_iterator> score_expression_grammar;
    score_expression_grammar grammar;
    client::score_expression ast;

    using boost::spirit::ascii::space;
    std::string::const_iterator iter = storage.begin();
    std::string::const_iterator saved = iter;
    std::string::const_iterator end = storage.end();
    bool r = phrase_parse(iter, end, grammar, space, ast);

    if (r && iter == end) {
        std::cout << "success!\n";
        std::cout << "what I parsed: " << ast << "\n";
        return 0;
    }
    else {
        std::string context(saved, iter);
        std::cout << "success!\n";
        std::cout << "how far I got: " << context << "\n";
        return 1;
    }

}
