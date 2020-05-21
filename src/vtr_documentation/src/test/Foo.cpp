#include <Foo.hpp>

namespace asrl {
namespace documentation {

Foo::Foo() {

}

void Foo::Bar(const int& var1,
              const int& var2,
              int& var3) {
  /// \todo Implement Bar. TODOs should be tagged so we can see them on the related todo page.
}

}
}