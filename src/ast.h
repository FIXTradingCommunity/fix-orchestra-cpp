#pragma once

// #define BOOST_SPIRIT_DEBUG

#include <boost/fusion/include/adapt_struct.hpp>

#include <boost/variant/recursive_wrapper.hpp>
#include <boost/variant/variant.hpp>

namespace score {
namespace ast {

struct OpUnaryMinus {};
struct OpLogicalNot {};
template <typename OpT> struct UnaryOp;

struct OpSub {};
struct OpAdd {};
struct OpDiv {};
struct OpMult {};
template <typename OpT> struct BinaryOp;

using Expr = boost::variant<unsigned int,
                            boost::recursive_wrapper<UnaryOp<OpUnaryMinus>>,
                            boost::recursive_wrapper<UnaryOp<OpLogicalNot>>,
                            boost::recursive_wrapper<BinaryOp<OpDiv>>,
                            boost::recursive_wrapper<BinaryOp<OpMult>>,
                            boost::recursive_wrapper<BinaryOp<OpSub>>,
                            boost::recursive_wrapper<BinaryOp<OpAdd>>>;

using Statement = boost::variant<Expr>;
}
}

namespace score {
namespace ast {

template <typename OpT> struct UnaryOp {
  explicit UnaryOp(const Expr &e) : expr(e) {}
  Expr expr;
};

template <typename OpT> struct BinaryOp {
  explicit BinaryOp(const Expr &le, const Expr &re) : lhs(le), rhs(re) {}
  Expr lhs;
  Expr rhs;
};
}
}

BOOST_FUSION_ADAPT_STRUCT(score::ast::UnaryOp<score::ast::OpLogicalNot>,
                          (score::ast::Expr, expr))
BOOST_FUSION_ADAPT_STRUCT(score::ast::UnaryOp<score::ast::OpUnaryMinus>,
                          (score::ast::Expr, expr))
BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryOp<score::ast::OpMult>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryOp<score::ast::OpDiv>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryOp<score::ast::OpAdd>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryOp<score::ast::OpSub>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))
