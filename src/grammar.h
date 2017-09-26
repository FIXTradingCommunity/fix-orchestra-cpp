#include <string>

namespace score {
namespace ast {
struct Statement {};
}
}

namespace score {
bool parse(ast::Statement &out, const std::string &in);
}
