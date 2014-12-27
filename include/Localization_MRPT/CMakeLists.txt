set(hdrs Localization_MRPT.h MonteCarloLocalization.h mrpt_localization_core.h
    PARENT_SCOPE
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

