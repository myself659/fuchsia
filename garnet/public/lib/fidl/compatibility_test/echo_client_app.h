// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Simple class that acts as a client of compatibility_test_service.Echo.
// In its own library so that both the C++ server and the compatibility test
// itself can use it.

#ifndef LIB_FIDL_COMPATIBILITY_TEST_ECHO_CLIENT_APP_H_
#define LIB_FIDL_COMPATIBILITY_TEST_ECHO_CLIENT_APP_H_

#include <fidl/test/compatibility/cpp/fidl.h>
#include <lib/sys/cpp/component_context.h>
#include <memory>
#include <string>

namespace fidl {
namespace test {
namespace compatibility {

class EchoClientApp {
 public:
  EchoClientApp();

  EchoPtr& echo();

  void Start(std::string server_url);

 private:
  EchoClientApp(const EchoClientApp&) = delete;
  EchoClientApp& operator=(const EchoClientApp&) = delete;

  std::unique_ptr<sys::ComponentContext> context_;
  std::shared_ptr<sys::ServiceDirectory> echo_provider_;
  fuchsia::sys::ComponentControllerPtr controller_;
  EchoPtr echo_;
};

}  // namespace compatibility
}  // namespace test
}  // namespace fidl

#endif  // LIB_FIDL_COMPATIBILITY_TEST_ECHO_CLIENT_APP_H_
