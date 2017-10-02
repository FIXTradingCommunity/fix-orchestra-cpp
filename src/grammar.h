#pragma once

#include <ast.h>

#include <string>

namespace score {
bool parse(ast::Statement &out, const std::string &in);
}
