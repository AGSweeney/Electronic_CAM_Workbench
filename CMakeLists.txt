# MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
cmake_minimum_required(VERSION 3.21)
project(ECamUI LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets Svg Network SerialPort PrintSupport)

# Cam model and motion laws: portable C++17 with no Qt, meant to be shared with the ClearCore firmware.
add_library(ecam_core STATIC
    core/CamLaws.h
    core/CamLaws.cpp
    core/CamProfile.h
    core/CamProfile.cpp
    core/ProfileText.h
    core/ProfileText.cpp
    core/CamRuntime.h
    core/CamRuntime.cpp
    core/CamGeometry.h
    core/CamGeometry.cpp
    core/CamTemplates.h
    core/CamTemplates.cpp
    core/CamPoints.h
    core/CamPoints.cpp
)
target_include_directories(ecam_core PUBLIC core)

# Excel workbook export (File > Export workbook): libxlsxwriter from vcpkg (BSD-2-Clause, plain C,
# no CMake package; vcpkg copies xlsxwriter.dll, zlib and minizip next to the exe).
find_path(XLSXWRITER_INCLUDE_DIR xlsxwriter.h REQUIRED)
find_library(XLSXWRITER_LIBRARY NAMES xlsxwriter REQUIRED)
add_library(ecam_workbook STATIC src/WorkbookExport.h src/WorkbookExport.cpp src/ProjectReview.h src/ProjectReview.cpp)
target_include_directories(ecam_workbook PUBLIC src PRIVATE ${XLSXWRITER_INCLUDE_DIR})
target_link_libraries(ecam_workbook PUBLIC ecam_core Qt6::Core PRIVATE ${XLSXWRITER_LIBRARY})

# Project report (File > Generate report): a vector PDF drawn with QPainter.
add_library(ecam_report STATIC src/ProjectReport.h src/ProjectReport.cpp)
target_link_libraries(ecam_report PUBLIC ecam_workbook Qt6::Gui Qt6::Svg)

# The application: ECamWorkbench.exe (Electronic CAM Workbench).
qt_add_executable(ECamWorkbench WIN32
    src/main.cpp
    src/Brand.h
    src/Theme.h
    src/Theme.cpp
    src/MainWindow.h
    src/MainWindow.cpp
    src/CamView.h
    src/CamView.cpp
    src/ProfileStrip.h
    src/ProfileStrip.cpp
    src/TimingView.h
    src/TimingView.cpp
    src/ProjectIo.h
    src/ProjectIo.cpp
    src/BoardLink.h
    src/BoardLink.cpp
    src/CollapsibleGroup.h
    src/CollapsibleGroup.cpp
    src/SideDock.h
    src/SideDock.cpp
    src/StartupUi.h
    src/StartupUi.cpp
    src/SetupWizard.h
    src/SetupWizard.cpp
    src/ClearPathLink.h
    src/ClearPathLink.cpp
    src/ScopeWindow.h
    src/ScopeWindow.cpp
    src/HelpWindow.h
    src/HelpWindow.cpp
    src/FirmwareUpdate.h
    src/FirmwareUpdate.cpp
    src/FirmwareDialog.h
    src/FirmwareDialog.cpp
    src/JogDialog.h
    src/JogDialog.cpp
    src/SwitchWizard.h
    src/SwitchWizard.cpp
    src/OperatorDialog.h
    src/OperatorDialog.cpp
    resources/app_resources.qrc
    resources/help/help.qrc
)
target_link_libraries(ECamWorkbench PRIVATE ecam_core ecam_workbook ecam_report Qt6::Widgets Qt6::Svg Qt6::Network Qt6::SerialPort Qt6::PrintSupport)

# Windows: embed the .ico via a real .rc (Explorer + pinned taskbar; RC_ICONS alone is flaky on MSVC).
# Regenerate the .ico from the SVG when the artwork changes (see README).
if (WIN32)
    set(_win_rc "${CMAKE_CURRENT_SOURCE_DIR}/resources/icons/ecam_app_icon.rc")
    target_sources(ECamWorkbench PRIVATE "${_win_rc}")
    set_source_files_properties("${_win_rc}" PROPERTIES
        OBJECT_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/resources/icons/ecam_app_icon.ico")
endif()

# Headless checks of the motion laws and the profile edits: ProjectData/build/<Config>/test_camlaws.exe
option(ECAMUI_BUILD_TESTS "Build test_camlaws" ON)
if (ECAMUI_BUILD_TESTS)
    add_executable(test_camlaws tests/test_camlaws.cpp)
    target_link_libraries(test_camlaws PRIVATE ecam_core)

    # The firmware's upload format and cam runtime, driven on the PC.
    add_executable(test_runtime tests/test_runtime.cpp)
    target_link_libraries(test_runtime PRIVATE ecam_core)

    # The firmware's EtherNet/IP assembly layout (EipProtocol.h), packed on the PC.
    add_executable(test_eip_protocol tests/test_eip_protocol.cpp)
    target_include_directories(test_eip_protocol PRIVATE Firmware/ECamFirmware)

    # Drives the real window with synthetic input (drag, table edit, law change, undo/redo).
    # ProjectData/build/<Config>/test_ui.exe (shows the window briefly)
    qt_add_executable(test_ui
        tests/test_ui.cpp
        src/Theme.h src/Theme.cpp
        src/MainWindow.h src/MainWindow.cpp
        src/CamView.h src/CamView.cpp
        src/ProfileStrip.h src/ProfileStrip.cpp
        src/TimingView.h src/TimingView.cpp
        src/ProjectIo.h src/ProjectIo.cpp
        src/BoardLink.h src/BoardLink.cpp
        src/CollapsibleGroup.h src/CollapsibleGroup.cpp
        src/SideDock.h src/SideDock.cpp
        src/StartupUi.h src/StartupUi.cpp
        src/SetupWizard.h src/SetupWizard.cpp
        src/ClearPathLink.h src/ClearPathLink.cpp
        src/ScopeWindow.h src/ScopeWindow.cpp
        src/HelpWindow.h src/HelpWindow.cpp
        src/FirmwareUpdate.h src/FirmwareUpdate.cpp
        src/FirmwareDialog.h src/FirmwareDialog.cpp
        src/JogDialog.h src/JogDialog.cpp
        src/SwitchWizard.h src/SwitchWizard.cpp src/OperatorDialog.h src/OperatorDialog.cpp
        resources/app_resources.qrc
        resources/help/help.qrc
    )
    target_include_directories(test_ui PRIVATE src)
    target_compile_definitions(test_ui PRIVATE ECAM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

    # Bench check against a real board (simulate mode, motors never enabled):
    # board_smoke <ip | COMn> samples/mixed_timeline.ecam
    qt_add_executable(board_smoke tests/board_smoke.cpp src/BoardLink.h src/BoardLink.cpp src/ProjectIo.h src/ProjectIo.cpp)
    target_include_directories(board_smoke PRIVATE src)
    target_link_libraries(board_smoke PRIVATE ecam_core Qt6::Core Qt6::Network Qt6::SerialPort)

    # The ClearPath scope link against a real motor on its own USB port (never enables or moves it):
    # scope_hw COM12 [seconds] [monitor variable id]
    qt_add_executable(scope_hw tests/scope_hw.cpp src/ClearPathLink.h src/ClearPathLink.cpp)
    target_include_directories(scope_hw PRIVATE src)
    target_link_libraries(scope_hw PRIVATE Qt6::Core Qt6::SerialPort)

    # The real scope window on the real drives: board_ui_scope <motors> [seconds] [shot.png] [names...]
    qt_add_executable(board_ui_scope tests/board_ui_scope.cpp
        src/Theme.h src/Theme.cpp
        src/ClearPathLink.h src/ClearPathLink.cpp
        src/ScopeWindow.h src/ScopeWindow.cpp
        src/HelpWindow.h src/HelpWindow.cpp)
    target_include_directories(board_ui_scope PRIVATE src)
    target_link_libraries(board_ui_scope PRIVATE Qt6::Widgets Qt6::Svg Qt6::SerialPort Qt6::PrintSupport)

    # The Axis jog panel against a real board: board_ui_axisjog <ip | COMn> <axis> [shot.png]
    # (enables and moves only that axis's motor; see the top of tests/board_ui_axisjog.cpp)
    qt_add_executable(board_ui_axisjog tests/board_ui_axisjog.cpp
        src/Theme.h src/Theme.cpp
        src/HelpWindow.h src/HelpWindow.cpp
        resources/help/help.qrc
        src/BoardLink.h src/BoardLink.cpp
        src/ProjectIo.h src/ProjectIo.cpp
        src/JogDialog.h src/JogDialog.cpp)
    target_include_directories(board_ui_axisjog PRIVATE src)
    target_link_libraries(board_ui_axisjog PRIVATE ecam_core Qt6::Widgets Qt6::Svg Qt6::Network Qt6::SerialPort Qt6::PrintSupport)

    # How well samples from several ClearPaths line up: scope_sync <seconds> <COMn> [COMn ...]
    qt_add_executable(scope_sync tests/scope_sync.cpp src/ClearPathLink.h src/ClearPathLink.cpp)
    target_include_directories(scope_sync PRIVATE src)
    target_link_libraries(scope_sync PRIVATE Qt6::Core Qt6::SerialPort)

    # pos_hw COM12: the drive's own commanded and measured position
    qt_add_executable(pos_hw tests/pos_hw.cpp src/ClearPathLink.h src/ClearPathLink.cpp)
    target_include_directories(pos_hw PRIVATE src)
    target_link_libraries(pos_hw PRIVATE Qt6::Core Qt6::SerialPort)

    # Hand jog through the real window against a board in simulate mode:
    # board_ui_jog <ip | COMn> samples/bench_driven_crank.ecam
    qt_add_executable(board_ui_jog
        tests/board_ui_jog.cpp
        src/Theme.h src/Theme.cpp
        src/MainWindow.h src/MainWindow.cpp
        src/CamView.h src/CamView.cpp
        src/ProfileStrip.h src/ProfileStrip.cpp
        src/TimingView.h src/TimingView.cpp
        src/ProjectIo.h src/ProjectIo.cpp
        src/BoardLink.h src/BoardLink.cpp
        src/CollapsibleGroup.h src/CollapsibleGroup.cpp
        src/SideDock.h src/SideDock.cpp
        src/StartupUi.h src/StartupUi.cpp
        src/SetupWizard.h src/SetupWizard.cpp
        src/ClearPathLink.h src/ClearPathLink.cpp
        src/ScopeWindow.h src/ScopeWindow.cpp
        src/HelpWindow.h src/HelpWindow.cpp
        src/FirmwareUpdate.h src/FirmwareUpdate.cpp
        src/FirmwareDialog.h src/FirmwareDialog.cpp
        src/JogDialog.h src/JogDialog.cpp
        src/SwitchWizard.h src/SwitchWizard.cpp src/OperatorDialog.h src/OperatorDialog.cpp
        resources/app_resources.qrc
        resources/help/help.qrc
    )
    target_include_directories(board_ui_jog PRIVATE src)
    target_link_libraries(board_ui_jog PRIVATE ecam_core ecam_workbook ecam_report Qt6::Widgets Qt6::Svg Qt6::Network Qt6::SerialPort Qt6::PrintSupport)

    # The Board panel with a PLC on EtherNet/IP (ecam_bench uitest drives the PLC side):
    # board_ui_plc <ip> samples/bench_driven_crank.ecam
    qt_add_executable(board_ui_plc
        tests/board_ui_plc.cpp
        src/Theme.h src/Theme.cpp
        src/MainWindow.h src/MainWindow.cpp
        src/CamView.h src/CamView.cpp
        src/ProfileStrip.h src/ProfileStrip.cpp
        src/TimingView.h src/TimingView.cpp
        src/ProjectIo.h src/ProjectIo.cpp
        src/BoardLink.h src/BoardLink.cpp
        src/CollapsibleGroup.h src/CollapsibleGroup.cpp
        src/SideDock.h src/SideDock.cpp
        src/StartupUi.h src/StartupUi.cpp
        src/SetupWizard.h src/SetupWizard.cpp
        src/ClearPathLink.h src/ClearPathLink.cpp
        src/ScopeWindow.h src/ScopeWindow.cpp
        src/HelpWindow.h src/HelpWindow.cpp
        src/FirmwareUpdate.h src/FirmwareUpdate.cpp
        src/FirmwareDialog.h src/FirmwareDialog.cpp
        src/JogDialog.h src/JogDialog.cpp
        src/SwitchWizard.h src/SwitchWizard.cpp src/OperatorDialog.h src/OperatorDialog.cpp
        resources/app_resources.qrc
        resources/help/help.qrc
    )
    target_include_directories(board_ui_plc PRIVATE src)
    target_link_libraries(board_ui_plc PRIVATE ecam_core ecam_workbook ecam_report Qt6::Widgets Qt6::Svg Qt6::Network Qt6::SerialPort Qt6::PrintSupport)

    # The Board page's Network settings against a real board: see the top of tests/board_ui_net.cpp
    qt_add_executable(board_ui_net
        tests/board_ui_net.cpp
        src/Theme.h src/Theme.cpp
        src/MainWindow.h src/MainWindow.cpp
        src/CamView.h src/CamView.cpp
        src/ProfileStrip.h src/ProfileStrip.cpp
        src/TimingView.h src/TimingView.cpp
        src/ProjectIo.h src/ProjectIo.cpp
        src/BoardLink.h src/BoardLink.cpp
        src/CollapsibleGroup.h src/CollapsibleGroup.cpp
        src/SideDock.h src/SideDock.cpp
        src/StartupUi.h src/StartupUi.cpp
        src/SetupWizard.h src/SetupWizard.cpp
        src/ClearPathLink.h src/ClearPathLink.cpp
        src/ScopeWindow.h src/ScopeWindow.cpp
        src/HelpWindow.h src/HelpWindow.cpp
        src/FirmwareUpdate.h src/FirmwareUpdate.cpp
        src/FirmwareDialog.h src/FirmwareDialog.cpp
        src/JogDialog.h src/JogDialog.cpp
        src/SwitchWizard.h src/SwitchWizard.cpp src/OperatorDialog.h src/OperatorDialog.cpp
        resources/app_resources.qrc
        resources/help/help.qrc
    )
    target_include_directories(board_ui_net PRIVATE src)
    target_link_libraries(board_ui_net PRIVATE ecam_core ecam_workbook ecam_report Qt6::Widgets Qt6::Svg Qt6::Network Qt6::SerialPort Qt6::PrintSupport)
    # The firmware update against a real ClearCore: see the top of tests/board_ui_fw.cpp
    qt_add_executable(board_ui_fw
        tests/board_ui_fw.cpp
        src/Theme.h src/Theme.cpp
        src/MainWindow.h src/MainWindow.cpp
        src/CamView.h src/CamView.cpp
        src/ProfileStrip.h src/ProfileStrip.cpp
        src/TimingView.h src/TimingView.cpp
        src/ProjectIo.h src/ProjectIo.cpp
        src/BoardLink.h src/BoardLink.cpp
        src/CollapsibleGroup.h src/CollapsibleGroup.cpp
        src/SideDock.h src/SideDock.cpp
        src/StartupUi.h src/StartupUi.cpp
        src/SetupWizard.h src/SetupWizard.cpp
        src/ClearPathLink.h src/ClearPathLink.cpp
        src/ScopeWindow.h src/ScopeWindow.cpp
        src/HelpWindow.h src/HelpWindow.cpp
        src/FirmwareUpdate.h src/FirmwareUpdate.cpp
        src/FirmwareDialog.h src/FirmwareDialog.cpp
        src/JogDialog.h src/JogDialog.cpp
        src/SwitchWizard.h src/SwitchWizard.cpp src/OperatorDialog.h src/OperatorDialog.cpp
        resources/app_resources.qrc
        resources/help/help.qrc
    )
    target_include_directories(board_ui_fw PRIVATE src)
    target_link_libraries(board_ui_fw PRIVATE ecam_core ecam_workbook ecam_report Qt6::Widgets Qt6::Svg Qt6::Network Qt6::SerialPort Qt6::PrintSupport)
    # The CCIO-8 page against a real board: see the top of tests/board_ui_ccio.cpp
    qt_add_executable(board_ui_ccio
        tests/board_ui_ccio.cpp
        src/Theme.h src/Theme.cpp
        src/MainWindow.h src/MainWindow.cpp
        src/CamView.h src/CamView.cpp
        src/ProfileStrip.h src/ProfileStrip.cpp
        src/TimingView.h src/TimingView.cpp
        src/ProjectIo.h src/ProjectIo.cpp
        src/BoardLink.h src/BoardLink.cpp
        src/CollapsibleGroup.h src/CollapsibleGroup.cpp
        src/SideDock.h src/SideDock.cpp
        src/StartupUi.h src/StartupUi.cpp
        src/SetupWizard.h src/SetupWizard.cpp
        src/ClearPathLink.h src/ClearPathLink.cpp
        src/ScopeWindow.h src/ScopeWindow.cpp
        src/HelpWindow.h src/HelpWindow.cpp
        src/FirmwareUpdate.h src/FirmwareUpdate.cpp
        src/FirmwareDialog.h src/FirmwareDialog.cpp
        src/JogDialog.h src/JogDialog.cpp
        src/SwitchWizard.h src/SwitchWizard.cpp src/OperatorDialog.h src/OperatorDialog.cpp
        resources/app_resources.qrc
        resources/help/help.qrc
    )
    target_include_directories(board_ui_ccio PRIVATE src)
    target_link_libraries(board_ui_ccio PRIVATE ecam_core ecam_workbook ecam_report Qt6::Widgets Qt6::Svg Qt6::Network Qt6::SerialPort Qt6::PrintSupport)
    target_link_libraries(test_ui PRIVATE ecam_core ecam_workbook ecam_report Qt6::Widgets Qt6::Svg Qt6::Network Qt6::SerialPort Qt6::PrintSupport)

    # The user guide's pictures from the real windows: help_images resources/help/images
    qt_add_executable(help_images
        tools/help_images/help_images.cpp
        src/Theme.h src/Theme.cpp
        src/MainWindow.h src/MainWindow.cpp
        src/CamView.h src/CamView.cpp
        src/ProfileStrip.h src/ProfileStrip.cpp
        src/TimingView.h src/TimingView.cpp
        src/ProjectIo.h src/ProjectIo.cpp
        src/BoardLink.h src/BoardLink.cpp
        src/CollapsibleGroup.h src/CollapsibleGroup.cpp
        src/SideDock.h src/SideDock.cpp
        src/StartupUi.h src/StartupUi.cpp
        src/SetupWizard.h src/SetupWizard.cpp
        src/ClearPathLink.h src/ClearPathLink.cpp
        src/ScopeWindow.h src/ScopeWindow.cpp
        src/HelpWindow.h src/HelpWindow.cpp
        src/FirmwareUpdate.h src/FirmwareUpdate.cpp
        src/FirmwareDialog.h src/FirmwareDialog.cpp
        src/JogDialog.h src/JogDialog.cpp
        src/SwitchWizard.h src/SwitchWizard.cpp src/OperatorDialog.h src/OperatorDialog.cpp
        resources/app_resources.qrc
        resources/help/help.qrc
    )
    target_include_directories(help_images PRIVATE src)
    target_compile_definitions(help_images PRIVATE ECAM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(help_images PRIVATE ecam_core ecam_workbook ecam_report Qt6::Widgets Qt6::Svg Qt6::Network Qt6::SerialPort Qt6::PrintSupport)
    # The video and document tools are kept out of the repository (.gitignore): built only where present.
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/ecam_demo/ecam_demo.cpp")
    # A scripted feature video: drives the real window with the real mouse (tools/ecam_demo)
    qt_add_executable(ecam_demo
        tools/ecam_demo/ecam_demo.cpp tools/ecam_demo/DemoKit.h tools/ecam_demo/video_waits.cpp tools/ecam_demo/video_laws.cpp tools/ecam_demo/video_firstcam.cpp tools/ecam_demo/video_jog.cpp tools/ecam_demo/video_homing.cpp tools/ecam_demo/video_operator.cpp tools/ecam_demo/video_plc.cpp tools/ecam_demo/video_masters.cpp tools/ecam_demo/video_cutlength.cpp
        src/Theme.h src/Theme.cpp
        src/MainWindow.h src/MainWindow.cpp
        src/CamView.h src/CamView.cpp
        src/ProfileStrip.h src/ProfileStrip.cpp
        src/TimingView.h src/TimingView.cpp
        src/ProjectIo.h src/ProjectIo.cpp
        src/BoardLink.h src/BoardLink.cpp
        src/CollapsibleGroup.h src/CollapsibleGroup.cpp
        src/SideDock.h src/SideDock.cpp
        src/StartupUi.h src/StartupUi.cpp
        src/SetupWizard.h src/SetupWizard.cpp
        src/ClearPathLink.h src/ClearPathLink.cpp
        src/ScopeWindow.h src/ScopeWindow.cpp
        src/HelpWindow.h src/HelpWindow.cpp
        src/FirmwareUpdate.h src/FirmwareUpdate.cpp
        src/FirmwareDialog.h src/FirmwareDialog.cpp
        src/JogDialog.h src/JogDialog.cpp
        src/SwitchWizard.h src/SwitchWizard.cpp src/OperatorDialog.h src/OperatorDialog.cpp
        resources/app_resources.qrc
        resources/help/help.qrc
    )
    target_include_directories(ecam_demo PRIVATE src)
    target_compile_definitions(ecam_demo PRIVATE ECAM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(ecam_demo PRIVATE ecam_core ecam_workbook ecam_report Qt6::Widgets Qt6::Svg Qt6::Network Qt6::SerialPort Qt6::PrintSupport)
    endif()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/docs_data/docs_data.cpp")
    # The theory document's chart data, from the core code (tools/docs_data -> docs/theory/data)
    qt_add_executable(docs_data tools/docs_data/docs_data.cpp src/ProjectIo.h src/ProjectIo.cpp)
    target_include_directories(docs_data PRIVATE src)
    target_link_libraries(docs_data PRIVATE ecam_core ecam_workbook ecam_report Qt6::Core Qt6::Gui)
    endif()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/ecam_anim/ecam_anim.cpp")
    # The feature videos' machine animations, from a project's own profile (tools/ecam_anim)
    qt_add_executable(ecam_anim tools/ecam_anim/ecam_anim.cpp src/ProjectIo.h src/ProjectIo.cpp)
    target_include_directories(ecam_anim PRIVATE src)
    target_link_libraries(ecam_anim PRIVATE ecam_core ecam_workbook ecam_report Qt6::Widgets)
    endif()
endif()
