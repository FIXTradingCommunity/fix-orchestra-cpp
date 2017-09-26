#pragma once

#include <boost/variant/recursive_wrapper.hpp>
#include <boost/variant/variant.hpp>

namespace score {
namespace ast {

struct OpUnaryMinus {};
struct OpLogicalNot {};
template <typename OpT> struct UnaryOp;

struct OpDiv {};
struct OpMult {};
template <typename OpT> struct BinaryOp;

using Expr =
    boost::variant<boost::recursive_wrapper<UnaryOp<OpUnaryMinus>>,
                   boost::recursive_wrapper<UnaryOp<OpLogicalNot>>,
                   boost::recursive_wrapper<BinaryOp<OpDiv>>,
                   boost::recursive_wrapper<BinaryOp<OpMult>>, unsigned int>;

struct Statement;
}
}

namespace score {
namespace ast {

template <typename OpT> struct UnaryOp { Expr expr; };

template <typename OpT> struct BinaryOp {
  Expr lhs;
  Expr rhs;
};

struct Statement {
  Expr expr;
};
}
}
