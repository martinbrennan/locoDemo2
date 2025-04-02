file (GLOB_RECURSE SRC_FILES "src/*c" "src/*cpp")
idf_component_register(
 SRCS ${SRC_FILES}
 PRIV_INCLUDE_DIRS "src" "src/utils" "src/libhelix-mp3"
 INCLUDE_DIRS ".")
                    
component_compile_options(-Wno-unused-variable -Wno-error=stringop-overflow)
