set(srcs "main.c" "nimble-spp-server.c")

idf_component_register(SRCS "${srcs}" INCLUDE_DIRS ".")
