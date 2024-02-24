// Copyright 2024 project-srs.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>
#include <test_lecture/calculator.hpp>

TEST(test_lecture, add_test)
{
  ASSERT_EQ(3, test_lecture::add(1, 2));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
