# CMake generated Testfile for 
# Source directory: C:/temp/pcre-8.01
# Build directory: C:/temp/pcre-8.01
# 
# This file includes the relevent testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(pcre_test "cmd" "/C" "C:/temp/pcre-8.01/RunTest.bat")
ADD_TEST(pcrecpp_test "C:/temp/pcre-8.01/DEBUG/pcrecpp_unittest.exe")
ADD_TEST(pcre_scanner_test "C:/temp/pcre-8.01/DEBUG/pcre_scanner_unittest.exe")
ADD_TEST(pcre_stringpiece_test "C:/temp/pcre-8.01/DEBUG/pcre_stringpiece_unittest.exe")
