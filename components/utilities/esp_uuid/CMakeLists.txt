#
# Versioning Information for ESP-IDF Components with GitHub, GitVersion and CMake
#
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
# Download CMake JSON-Parser: https://github.com/sbellus/json-cmake/blob/master/JSONParser.cmake
# Copy the CMake JSONParser.cmake file to the tools/cmake directory of your ESP-IDF installation.
# i.e. C:\Users\user\.platformio\packages\framework-espidf\tools\cmake
#
include($ENV{IDF_PATH}/tools/cmake/version.cmake)

# check if JSONParser is available
find_package(JSONParser)

# test
if(JSONPARSER_FOUND)
    message("JSON Parser Found")
else()
    message("JSON Parser Not Found")
endif()

# validate JSONParser and version.h.in file are available
if(JSONPARSER_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vers/version.h.in")
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
    set(GIT_VERSION_DATE  ${GIT_VERSION_JSON.CommitDate})
    set(GIT_SEM_VERSION   ${GIT_VERSION_JSON.MajorMinorPatch})
    set(GIT_VERSION_MAJOR ${GIT_VERSION_JSON.Major})
    set(GIT_VERSION_MINOR ${GIT_VERSION_JSON.Minor})
    set(GIT_VERSION_PATCH ${GIT_VERSION_JSON.Patch})
    set(GIT_FULL_SEM_VER  ${GIT_VERSION_JSON.FullSemVer})
    set(GIT_SHORT_SHA     ${GIT_VERSION_JSON.ShortSha})

    # Release json variable
    sbeClearJson(GIT_VERSION_JSON)

    # Parse component name from component path
    get_filename_component(COMPONENT_NAME "${COMPONENT_DIR}" NAME)

    # Components should be named as "esp_<component_name>"
    string(FIND "${COMPONENT_NAME}" "esp_" ESP_PREFIX)

    # Check if the component name starts with "esp_"
    if(ESP_PREFIX EQUAL -1)
        # Use the component name as is
        string(CONCAT COMPONENT_FILE_NAME "" "${COMPONENT_NAME}")
    else()
        # Parse component file name from component name
        string(REPLACE "esp_" "" COMPONENT_FILE_NAME "${COMPONENT_NAME}")
    endif()

    # Set the component file name to upper case
    string(TOUPPER "${COMPONENT_FILE_NAME}" COMPONENT_FILE_NAME_UPPER)

    # Generate header file with versioning information
    configure_file( "${CMAKE_SOURCE_DIR}/vers/version.h.in" "${COMPONENT_DIR}/include/${COMPONENT_FILE_NAME}_version.h" @ONLY )

    # Update YAML & JSON files with versioning information
    execute_process(
        COMMAND python3 "${CMAKE_SOURCE_DIR}/vers/cmake_build.py" "${COMPONENT_DIR}" "${GIT_SEM_VERSION}"
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        ERROR_VARIABLE ERROR
        OUTPUT_VARIABLE  OUTPUT
        COMMAND_ERROR_IS_FATAL ANY
    )

    message("Output Var: " ${OUTPUT})
    message("Error Var:  " ${ERROR})
endif()

# Register the component
idf_component_register(
    SRCS uuid.c
    INCLUDE_DIRS include
)
