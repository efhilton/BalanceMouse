idf_component_register(
        SRCS
        "Main.cpp"
        "Qmi8658C.cpp"

        INCLUDE_DIRS
        ".."
        "../include"

        REQUIRES
        "driver"
)
