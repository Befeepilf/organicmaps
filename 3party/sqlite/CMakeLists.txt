add_library(sqlite STATIC
  sqlite3.c
  sqlite3.h
)

target_include_directories(sqlite PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}"
)