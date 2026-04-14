TEMPLATE = app

CONFIG += debug_and_release qt
CONFIG += console
QT += xml
QT += core gui xml
TARGET = Tutorial

isEmpty(CSF_OCCTSamplesPath) {
    SAMPLESROOT = $$PWD/..
} else {
    SAMPLESROOT = $$quote($$(CSF_OCCTSamplesPath)/qt)
}

HEADERS   = src/*.h \
            $${SAMPLESROOT}/Common/src/*.h \
            $${SAMPLESROOT}/Interface/src/*.h

SOURCES   = src/*.cxx \
            $${SAMPLESROOT}/Common/src/*.cxx \
            $${SAMPLESROOT}/Interface/src/*.cxx

TS_FILES  = $${SAMPLESROOT}/Common/src/Common-icon.ts \
            $${SAMPLESROOT}/Common/src/Common-string.ts \
            ./src/Tutorial-icon.ts \
            ./src/Tutorial-string.ts

RES_FILES = $${SAMPLESROOT}/Common/res/* \
            ./res/*

INCLUDEPATH += $$quote($${SAMPLESROOT}/Common/src)
INCLUDEPATH += $$quote($${SAMPLESROOT}/Interface/src)
INCLUDEPATH += $$quote($$(CSF_OCCTIncludePath))
INCLUDEPATH += D:/OCC/GC-OCC/OCCT-7_9_3/inc
INCLUDEPATH += D:/OCC/GC-OCC/rl0.7.0/include/rl-0.7.0
INCLUDEPATH += D:/OCC/GC-OCC/rl0.7.0/include/eigen3
INCLUDEPATH += D:/OCC/GC-OCC/rl0.7.0/include

OCCT_DEFINES = $$(CSF_DEFINES)
message(OCCT_DEFINES = $$(CSF_DEFINES))
DEFINES = $$split(OCCT_DEFINES, ;)

message(${QMAKE_FILE_IN})
message($$(QTDIR))



win32 {
    CONFIG(debug, debug|release) {
        DEFINES += _DEBUG
        DESTDIR = ./win$$(ARCH)/$$(VCVER)/bind
        OBJECTS_DIR = ./win$$(ARCH)/$$(VCVER)/objd
        MOC_DIR = ./win$$(ARCH)/$$(VCVER)/mocd
        #      ָ    2026/2/4 13:45:01             ļ     ȡexeͬ ļ    µ      ļ
        RES_DIR   = ./win$$(ARCH)/$$(VCVER)/bind/samples
    } else {
        DEFINES += NDEBUG
        DESTDIR = ./win$$(ARCH)/$$(VCVER)/bin
        OBJECTS_DIR = ./win$$(ARCH)/$$(VCVER)/obj
        MOC_DIR = ./win$$(ARCH)/$$(VCVER)/moc
        RES_DIR   = ./win$$(ARCH)/$$(VCVER)/bin/samples
    }
    LIBS = -L$$(QTDIR)/lib;$$(CSF_OCCTLibPath)
    QMAKE_CXXFLAGS += /std:c++17
    DEFINES += NO_COMMONSAMPLE_EXPORTS NO_IESAMPLE_EXPORTS
}

LIBS += -LD:/OCC/GC-OCC/OCCT-7_9_3/win64/vc14/lib \
                -lTKernel -lTKMath -lTKService -lTKV3d -lTKOpenGl \
        -lTKBRep -lTKDEIGES -lTKDESTL -lTKDEVRML -lTKDESTEP \
        -lTKGeomBase -lTKGeomAlgo -lTKG3d -lTKG2d \
        -lTKXSBase -lTKShHealing -lTKHLR -lTKTopAlgo -lTKMesh -lTKPrim \
        -lTKCDF -lTKBool -lTKBO -lTKFillet -lTKOffset -lTKLCAF

LIBS += -LD:/OCC/GC-OCC/rl0.7.0/lib \
     -lrlmdls \
     -llibxml2 \
     -llibxslt \
     -lrlkins

message(RES_DIR = $$(RES_DIR))
!exists($${RES_DIR}) {
    win32 {
#        system(mkdir $${RES_DIR})
    } else {
        system(mkdir -p $${RES_DIR})
    }
}

TRANSLATIONS += translations/app_en_US.ts
                #translations/app_zh_CN.ts


lrelease.name = LRELEASE ${QMAKE_FILE_IN}
lrelease.commands = lrelease ${QMAKE_FILE_IN} -qm $${RES_DIR}/${QMAKE_FILE_BASE}.qm
lrelease.output = ${QMAKE_FILE_BASE}.qm
lrelease.input = TS_FILES
lrelease.clean = $${RES_DIR}/${QMAKE_FILE_BASE}.qm
lrelease.CONFIG += no_link target_predeps
QMAKE_EXTRA_COMPILERS += lrelease

copy_res.name = Copy resource ${QMAKE_FILE_IN}
copy_res.output = ${QMAKE_FILE_BASE}${QMAKE_FILE_EXT}
copy_res.clean = $${RES_DIR}/${QMAKE_FILE_BASE}${QMAKE_FILE_EXT}
copy_res.input = RES_FILES
copy_res.CONFIG += no_link target_predeps
win32: copy_res.commands = type ${QMAKE_FILE_IN} > $${RES_DIR}/${QMAKE_FILE_BASE}${QMAKE_FILE_EXT}
unix:  copy_res.commands = cp -f ${QMAKE_FILE_IN} $${RES_DIR}
QMAKE_EXTRA_COMPILERS += copy_res
#QMAKE_CXXFLAGS += /wd4996

greaterThan(QT_MAJOR_VERSION, 4) {
    QT += widgets
}

message("Qmake ompleted")

