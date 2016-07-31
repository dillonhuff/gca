#define CATCH_CONFIG_RUNNER

#include <iostream>

#include "catch.hpp"

int main( int argc, char* const argv[] ) {

  Catch::Timer timer;
  timer.start();

  int result = Catch::Session().run( argc, argv );

  std::cout << "runtime: " << timer.getElapsedSeconds() <<  " seconds " << std::endl;

  return result;
}
