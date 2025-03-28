# Inspired by: https://www.esp32.com/viewtopic.php?f=2&t=45054&p=146150#p146150
#
# Install Git-Version via command prompt: dotnet tool install --global GitVersion.Tool
# Create a GitVersion.yml file in the root of your project with the following content:
#
#   major-version-bump-message: '\+semver:\s?(breaking|major)'
#   minor-version-bump-message: '\+semver:\s?(feature|minor)'
#   patch-version-bump-message: '\+semver:\s?(fix|patch)'
#   commit-message-incrementing: Enabled
#
# CMake JSON-Parser: https://github.com/sbellus/json-cmake/blob/master/JSONParser.cmake
# Copy the JSONParser.cmake file to the tools/cmake directory of your ESP-IDF installation.
# i.e. C:\Users\user\.platformio\packages\framework-espidf\tools\cmake
#
include($ENV{IDF_PATH}/tools/cmake/version.cmake)
include($ENV{IDF_PATH}/tools/cmake/JSONParser.cmake)

# Get latest versioning information from git repository
execute_process(
    COMMAND dotnet-gitversion
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION_OUTPUT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Instantiate json variable
sbeParseJson(GIT_VERSION_JSON GIT_VERSION_OUTPUT)

# Parse versioning variables from json output
set(GIT_VERSION_DATE ${GIT_VERSION_JSON.CommitDate})
set(GIT_VERSION_MAJOR ${GIT_VERSION_JSON.Major})
set(GIT_VERSION_MINOR ${GIT_VERSION_JSON.Minor})
set(GIT_VERSION_PATCH ${GIT_VERSION_JSON.Patch})
set(GIT_FULL_SEM_VER ${GIT_VERSION_JSON.FullSemVer})
set(GIT_SHORT_SHA ${GIT_VERSION_JSON.ShortSha})

# Release json variable
sbeClearJson(GIT_VERSION_JSON)

# Generate a header file with version information
configure_file( "${CMAKE_SOURCE_DIR}/vers/version.h.in" "${COMPONENT_DIR}/include/version.h" @ONLY )

# Register the component
idf_component_register(
    SRCS type_utils.c
    INCLUDE_DIRS include
    REQUIRES esp_common
)
