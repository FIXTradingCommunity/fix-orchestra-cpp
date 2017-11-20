#pragma once

// #define BOOST_SPIRIT_DEBUG

// required to support large variant structure of expr
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_LIST_SIZE 40
#define BOOST_MPL_LIMIT_VECTOR_SIZE 40

#include <boost/optional.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/fusion/include/adapt_struct.hpp>

#include <boost/variant/recursive_wrapper.hpp>
#include <boost/variant/variant.hpp>

#include <string>
#include <vector>

namespace score {
namespace ast {

// TODO
// struct Period {
// };
}
}

namespace score {
namespace ast {

struct OpUnaryMinus {};
struct OpLogicalNot {};
template <typename OpT> struct UnaryOp;
struct OpSub {};
struct OpAdd {};
struct OpMod {};
struct OpDiv {};
struct OpMult {};
template <typename OpT> struct BinaryOp;
struct RelGreaterThanEq {};
struct RelGreaterThan {};
struct RelLessThanEq {};
struct RelLessThan {};
struct RelNotEq {};
struct RelEq {};
struct RelAnd {};
struct RelOr {};
template <typename OpT> struct BinaryRel;
struct Range;
struct Contains;

struct Predicate;
struct Qualifier;
struct Variable;
struct Exists;

struct Assignment;

using Expr = boost::variant<
    char,
    unsigned int, // list these first since first field needs to be default
                  // constructible
    double,       // TODO: not exact/arb precision - will improve later
    std::string, boost::gregorian::date, boost::posix_time::time_duration,
    boost::posix_time::ptime, // Period,
    boost::recursive_wrapper<Variable>, boost::recursive_wrapper<Exists>,

    boost::recursive_wrapper<UnaryOp<OpUnaryMinus>>,
    boost::recursive_wrapper<UnaryOp<OpLogicalNot>>,
    boost::recursive_wrapper<BinaryOp<OpMod>>,
    boost::recursive_wrapper<BinaryOp<OpDiv>>,
    boost::recursive_wrapper<BinaryOp<OpMult>>,
    boost::recursive_wrapper<BinaryOp<OpSub>>,
    boost::recursive_wrapper<BinaryOp<OpAdd>>,
    boost::recursive_wrapper<BinaryRel<RelNotEq>>,
    boost::recursive_wrapper<BinaryRel<RelLessThan>>,
    boost::recursive_wrapper<BinaryRel<RelLessThanEq>>,
    boost::recursive_wrapper<BinaryRel<RelGreaterThan>>,
    boost::recursive_wrapper<BinaryRel<RelGreaterThanEq>>,
    boost::recursive_wrapper<Range>, boost::recursive_wrapper<Contains>,
    boost::recursive_wrapper<BinaryRel<RelEq>>,
    boost::recursive_wrapper<BinaryRel<RelAnd>>,
    boost::recursive_wrapper<BinaryRel<RelOr>>>;

using Statement = boost::variant<boost::recursive_wrapper<Assignment>, Expr>;
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
  Expr lhs, rhs;
};

template <typename OpT> struct BinaryRel {
  explicit BinaryRel(const Expr &le, const Expr &re) : lhs(le), rhs(re) {}
  Expr lhs, rhs;
};

struct Range {
  explicit Range(const Expr &elhs, const Expr &elb, const Expr &eub)
      : lhs(elhs), lb(elb), ub(eub) {}
  Expr lhs, lb, ub;
};

struct Contains {
  explicit Contains(const Expr &elhs, const std::vector<Expr> &eset)
      : lhs(elhs), set(eset) {}
  Expr lhs;
  std::vector<Expr> set;
};

struct Predicate {
  explicit Predicate() = default;
  explicit Predicate(const std::string &pid, const Expr &e)
      : id(pid), expr(e) {}
  std::string id;
  Expr expr;
};

using QualInnerT = boost::optional<boost::variant<unsigned int, Predicate>>;
struct Qualifier {
  explicit Qualifier() = default;
  explicit Qualifier(const std::string &qid, const QualInnerT &q)
      : id(qid), qual(q) {}
  std::string id;
  QualInnerT qual;
};

struct Variable {
  explicit Variable() = default;
  explicit Variable(boost::optional<std::string> &s,
                    const std::vector<Qualifier> &qs)
      : scope(s), quals(qs) {}
  boost::optional<std::string>
      scope; // TODO: convert this to an enum during construction
  std::vector<Qualifier> quals;
};

struct Exists {
  explicit Exists(const Variable &v) : var(v) {}
  Variable var;
};

struct Assignment {
  explicit Assignment() = default;
  explicit Assignment(const Variable &l, const Expr &r) : lhs(l), rhs(r) {}
  Variable lhs;
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
BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryOp<score::ast::OpMod>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryOp<score::ast::OpDiv>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryOp<score::ast::OpAdd>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryOp<score::ast::OpSub>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))

BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryRel<score::ast::RelNotEq>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryRel<score::ast::RelLessThan>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryRel<score::ast::RelLessThanEq>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryRel<score::ast::RelGreaterThan>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryRel<score::ast::RelGreaterThanEq>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))

BOOST_FUSION_ADAPT_STRUCT(score::ast::Range,
                          (score::ast::Expr, lhs)(score::ast::Expr,
                                                  lb)(score::ast::Expr, ub))
BOOST_FUSION_ADAPT_STRUCT(score::ast::Contains,
                          (score::ast::Expr, lhs)(std::vector<score::ast::Expr>,
                                                  set))

BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryRel<score::ast::RelEq>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))

BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryRel<score::ast::RelAnd>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))
BOOST_FUSION_ADAPT_STRUCT(score::ast::BinaryRel<score::ast::RelOr>,
                          (score::ast::Expr, lhs)(score::ast::Expr, rhs))

BOOST_FUSION_ADAPT_STRUCT(score::ast::Predicate,
                          (std::string, id)(score::ast::Expr, expr))
BOOST_FUSION_ADAPT_STRUCT(score::ast::Qualifier,
                          (std::string, id)(score::ast::QualInnerT, qual))
BOOST_FUSION_ADAPT_STRUCT(score::ast::Variable,
                          (boost::optional<std::string>,
                           scope)(std::vector<score::ast::Qualifier>, quals))
BOOST_FUSION_ADAPT_STRUCT(score::ast::Exists, (score::ast::Variable, var))

BOOST_FUSION_ADAPT_STRUCT(score::ast::Assignment,
                          (score::ast::Variable, lhs)(score::ast::Expr, rhs))
