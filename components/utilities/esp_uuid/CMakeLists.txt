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
include($ENV{IDF_PATH}/tools/cmake/JSONParser.cmake RESULT_VARIABLE JSONPARSER_FOUND)

# string compare JSONParser library availability
string(COMPARE NOTEQUAL "${JSONPARSER_FOUND}" "NOTFOUND" STR_CMP_RESULT)

# validate JSONParser library, version.h.in, pio_lib_sync.py, esp_cmp_sync.py, 
# library.json.in, and idf_component.yml.in files are available for preprocessing
if(STR_CMP_RESULT EQUAL 1 AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/stg/version.h.in" 
    AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/stg/pio_lib_sync.py" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/stg/esp_cmp_sync.py"
    AND EXISTS "${COMPONENT_DIR}/include/library.json.in" AND EXISTS "${COMPONENT_DIR}/include/idf_component.yml.in")
    
    # Get latest versioning information from git repository with GitVersion
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

    # Generate C header file from template with versioning information
    configure_file( "${CMAKE_SOURCE_DIR}/stg/version.h.in" "${COMPONENT_DIR}/include/${COMPONENT_FILE_NAME}_version.h" @ONLY )

    # Generate json library file from template with versioning information
    configure_file( "${COMPONENT_DIR}/include/library.json.in" "${COMPONENT_DIR}/library.json" @ONLY )

    # Generate yml idf component file from template with versioning information
    configure_file( "${COMPONENT_DIR}/include/idf_component.yml.in" "${COMPONENT_DIR}/idf_component.yml" @ONLY )

    # Package components and upload to ESP-IDF component and platformio 
    # library registries when the version is tagged with a release version
    #execute_process(
    #    COMMAND python "${CMAKE_SOURCE_DIR}/vers/cmake_build.py" "${COMPONENT_DIR}" "${GIT_SEM_VERSION}"
    #    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    #    RESULT_VARIABLE PY_RESULT
    #)
endif()

# Register the component
idf_component_register(
    SRCS uuid.c
    INCLUDE_DIRS include
)
