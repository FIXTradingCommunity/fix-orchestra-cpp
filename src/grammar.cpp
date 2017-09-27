#include <grammar.h>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/support.hpp>

namespace score {

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
namespace phx = boost::phoenix;

template <typename Iterator>
struct Grammar : qi::grammar<Iterator, ast::Statement(), ascii::space_type> {
  Grammar() : Grammar::base_type(statementRule) {
    using namespace qi;

    statementRule = exprRule.alias() >> eoi;

    exprRule      = orRule.alias();
    orRule
      = andRule[_val = _1]
        >> *(((lit("or") | lit("||")) >> andRule) [_val = phx::construct<ast::BinaryRel<ast::RelOr>>(_val, _1)]);
    andRule
      = eqRule[_val = _1]
        >> *(((lit("and") | lit("&&")) >> eqRule) [_val = phx::construct<ast::BinaryRel<ast::RelAnd>>(_val, _1)]);
    eqRule
      = inclusionRule[_val = _1]
        >> *( ((lit("eq") | lit("==")) >> inclusionRule) [_val = phx::construct<ast::BinaryRel<ast::RelEq>>(_val, _1)]
            | ((lit("ne") | lit("!=")) >> inclusionRule) [_val = phx::construct<ast::BinaryRel<ast::RelNotEq>>(_val, _1)]);
    inclusionRule // TODO: does this take precedence over `relRule`s? we should think about this
      = relRule[_val = _1]
        >> *( (lit("between") >> lit("[") >> relRule >> lit(",") >> relRule >> lit("]")) [_val = phx::construct<ast::Range>(_val, _1, _2)] // XXX: differs from ANTLR grammar!
            | (lit("in") >> lit("{") >> (relRule % ',') >> lit("}")) [_val = phx::construct<ast::Contains>(_val, _1)]);
    relRule
      = addSubRule[_val = _1]
        >> *( ((lit("<") | lit("lt")) >> addSubRule) [_val = phx::construct<ast::BinaryRel<ast::RelLessThan>>(_val, _1)]
            | ((lit("<=") | lit("le")) >> addSubRule) [_val = phx::construct<ast::BinaryRel<ast::RelLessThanEq>>(_val, _1)]
            | ((lit(">") | lit("gt")) >> addSubRule) [_val = phx::construct<ast::BinaryRel<ast::RelGreaterThan>>(_val, _1)]
            | ((lit(">=") | lit("ge")) >> addSubRule) [_val = phx::construct<ast::BinaryRel<ast::RelGreaterThanEq>>(_val, _1)]);
    addSubRule
      = mulDivRule[_val = _1]
        >> *( (lit('+') >> mulDivRule) [_val = phx::construct<ast::BinaryOp<ast::OpAdd>>(_val, _1)]
            | (lit('-') >> mulDivRule) [_val = phx::construct<ast::BinaryOp<ast::OpSub>>(_val, _1)]);
    mulDivRule
      = unaryRule[_val = _1]
        >> *( (lit('*') >> unaryRule) [_val = phx::construct<ast::BinaryOp<ast::OpMult>>(_val, _1)]
            | (lit('/') >> unaryRule) [_val = phx::construct<ast::BinaryOp<ast::OpDiv>>(_val, _1)]
            | ((lit("mod") | lit("%")) >> unaryRule) [_val = phx::construct<ast::BinaryOp<ast::OpMod>>(_val, _1)]);
    unaryRule // TODO: better to use '>' instead of '>>' ... but throws exception on bad parse
      = (lit('-') >> exprRule) [_val = phx::construct<ast::UnaryOp<ast::OpUnaryMinus>>(_1)]
      | (lit('!') >> exprRule) [_val = phx::construct<ast::UnaryOp<ast::OpLogicalNot>>(_1)]
      | simpleRule            [_val = _1];
    simpleRule
      = (lit('(') >> exprRule >> lit(')')) [_val = _1]
      | uint_                              [_val = _1];

    BOOST_SPIRIT_DEBUG_NODE(exprRule);
    BOOST_SPIRIT_DEBUG_NODE(unaryRule);
    BOOST_SPIRIT_DEBUG_NODE(mulDivRule);
    BOOST_SPIRIT_DEBUG_NODE(simpleRule);
    BOOST_SPIRIT_DEBUG_NODE(statementRule);
  }

  qi::rule<Iterator, ast::Expr(),      ascii::space_type> exprRule,
                                                          orRule,
                                                          andRule,
                                                          eqRule,
                                                          inclusionRule,
                                                          relRule,
                                                          addSubRule,
                                                          mulDivRule,
                                                          unaryRule,
                                                          simpleRule;
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
