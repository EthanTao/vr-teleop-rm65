#pragma once
#include <string>
namespace std_srvs { namespace srv { struct Trigger {
struct Request {}; struct Response { bool success=false; std::string message; };
}; }}
