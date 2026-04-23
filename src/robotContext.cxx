#include "DL_RobotContext.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>

#include <AIS_ListOfInteractive.hxx>
#include <AIS_Trihedron.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRep_Builder.hxx>
#include <Geom_Axis2Placement.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESControl_Reader.hxx>
#include <Prs3d_DatumAspect.hxx>
#include <Quantity_Color.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Iterator.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <QCoreApplication>
#include <QDialog>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPixmap>
#include <QPushButton>
#include <QRadioButton>
#include <QStringList>
#include <QTextStream>
#include <QVBoxLayout>

#include <rl/math/Unit.h>
#include <rl/mdl/Joint.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/NloptInverseKinematics.h>
#include <rl/mdl/XmlFactory.h>

namespace
{
// RL 运动学模板仍然作为运行时底座存在，Top.xml 只是更上层的入口描述。
static const char* ROBOT_XML_TEMPLATE = R"(<?xml version="1.0" encoding="UTF-8"?><rlmdl xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="rlmdl.xsd"><model><manufacturer>Huashu</manufacturer><name>HSR-CR630-1750</name><world id="world"><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0</z></translation><g><x>0</x><y>0</y><z>9.86055</z></g></world><body id="body0"><ignore/><ignore idref="body1"/></body><frame id="frame0"/><frame id="frame1"/><body id="body1"><ignore idref="body0"/><ignore idref="body2"/></body><frame id="frame2"/><body id="body2"><ignore idref="body1"/><ignore idref="body3"/><ignore idref="body4"/></body><frame id="frame3"/><body id="body3"><ignore idref="body2"/><ignore idref="body4"/></body><frame id="frame4"/><body id="body4"><ignore idref="body2"/><ignore idref="body3"/><ignore idref="body5"/><ignore idref="body6"/></body><frame id="frame5"/><body id="body5"><ignore idref="body4"/><ignore idref="body6"/></body><frame id="frame6"/><body id="body6"><ignore idref="body4"/><ignore idref="body5"/></body><frame id="frame7"/><fixed id="fixed0"><frame><a idref="world"/><b idref="body0"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0</z></translation></fixed><fixed id="fixed1"><frame><a idref="body0"/><b idref="frame0"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0</z></translation></fixed><revolute id="joint0"><frame><a idref="frame0"/><b idref="frame1"/></frame><axis><x>0</x><y>0</y><z>1</z></axis><max>360</max><min>-360</min><speed>187</speed></revolute><fixed id="fixed2"><frame><a idref="frame1"/><b idref="body1"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0.5015</z></translation></fixed><revolute id="joint1"><frame><a idref="body1"/><b idref="frame2"/></frame><axis><x>0</x><y>1</y><z>0</z></axis><max>360</max><min>-360</min><speed>160</speed></revolute><fixed id="fixed3"><frame><a idref="frame2"/><b idref="body2"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0.950</z></translation></fixed><revolute id="joint2"><frame><a idref="body2"/><b idref="frame3"/></frame><axis><x>0</x><y>1</y><z>0</z></axis><max>360</max><min>-360</min><speed>180</speed></revolute><fixed id="fixed4"><frame><a idref="frame3"/><b idref="body3"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0</z></translation></fixed><revolute id="joint3"><frame><a idref="body3"/><b idref="frame4"/></frame><axis><x>1</x><y>0</y><z>0</z></axis><max>360</max><min>-360</min><speed>260</speed></revolute><fixed id="fixed5"><frame><a idref="frame4"/><b idref="body4"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0.615</x><y>0</y><z>0</z></translation></fixed><revolute id="joint4"><frame><a idref="body4"/><b idref="frame5"/></frame><axis><x>0</x><y>1</y><z>0</z></axis><max>360</max><min>-360</min><speed>230</speed></revolute><fixed id="fixed6"><frame><a idref="frame5"/><b idref="body5"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>-0.177</y><z>-0.185</z></translation></fixed><revolute id="joint5"><frame><a idref="body5"/><b idref="frame6"/></frame><axis><x>0</x><y>0</y><z>1</z></axis><max>360</max><min>-360</min><speed>360</speed></revolute><fixed id="fixed7"><frame><a idref="frame6"/><b idref="body6"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0</z></translation></fixed><fixed id="fixed8"><frame><a idref="body6"/><b idref="frame7"/></frame><rotation><x>0</x><y>0</y><z>0</z></rotation><translation><x>0</x><y>0</y><z>0</z></translation></fixed></model></rlmdl>)";
// 当 Top.xml 未显式提供杆件名称时，使用这组默认名称填充。
static const char* DEFAULT_ROD_NAMES[DL_ROBOT_JOINT_COUNT + 1] =
{
    "Base", "Shoulder", "UpperArm", "ForeArm", "Wrist1", "Wrist2", "Flange"
};

// 获取或补建子节点，避免 XML 写入阶段重复判断节点是否存在。
QDomElement ensureChildElement(QDomDocument& theDocument, QDomElement theParent, const QString& theTag)
{
    QDomElement aNode = theParent.firstChildElement(theTag);
    if (aNode.isNull())
    {
        aNode = theDocument.createElement(theTag);
        theParent.appendChild(aNode);
    }
    return aNode;
}

// 统一更新文本节点内容，供 Top.xml -> robot.xml 映射过程复用。
void updateTextNode(QDomDocument& theDocument, QDomElement theParent, const QString& theTag, const QString& theValue)
{
    QDomElement aNode = ensureChildElement(theDocument, theParent, theTag);
    if (aNode.firstChild().isNull())
        aNode.appendChild(theDocument.createTextNode(theValue));
    else
        aNode.firstChild().setNodeValue(theValue);
}

// 按固定关节 id 回写 translation，保持与既有 RL 模板结构一致。
void setFixedTranslation(QDomDocument& theDocument,
                         const QString& theId,
                         const QString& theX,
                         const QString& theY,
                         const QString& theZ)
{
    QDomNodeList aNodes = theDocument.elementsByTagName("fixed");
    for (int i = 0; i < aNodes.count(); ++i)
    {
        QDomElement aFixed = aNodes.at(i).toElement();
        if (aFixed.attribute("id") != theId) continue;
        QDomElement aTranslation = ensureChildElement(theDocument, aFixed, "translation");
        updateTextNode(theDocument, aTranslation, "x", theX);
        updateTextNode(theDocument, aTranslation, "y", theY);
        updateTextNode(theDocument, aTranslation, "z", theZ);
        return;
    }
}

// 按两种手腕姿态给出默认轴向、限位和速度，后续允许被 Top.xml 或 UI 输入覆盖。
void fillJointDefaults(bool theIsPose1,
                       QStringList& theAxisX,
                       QStringList& theAxisY,
                       QStringList& theAxisZ,
                       QStringList& theMinVals,
                       QStringList& theMaxVals,
                       QStringList& theSpeedVals)
{
    theAxisX = QStringList() << "0" << "0" << "0" << "1" << "0" << "1";
    theAxisY = QStringList() << "0" << "1" << "1" << "0" << "1" << "0";
    theAxisZ = QStringList() << "1" << "0" << "0" << "0" << "0" << "0";
    if (!theIsPose1)
    {
        theAxisX[5] = "0";
        theAxisZ[5] = "1";
    }

    theMinVals = QStringList() << "-360" << "-360" << "-360" << "-360" << "-360" << "-360";
    theMaxVals = QStringList() << "360" << "360" << "360" << "360" << "360" << "360";
    theSpeedVals = QStringList() << "187" << "160" << "180" << "260" << "230" << "360";
}

// 将 Top.xml 中的关键参数映射回 RL 运行时 robot.xml，尽量复用现有运动学链路。
bool buildRobotMdlDocument(const QString& theManufacturer,
                           const QString& theModelName,
                           const bool     theIsPose1,
                           const QStringList& theDimensions,
                           const QStringList& theAxisX,
                           const QStringList& theAxisY,
                           const QStringList& theAxisZ,
                           const QStringList& theMinVals,
                           const QStringList& theMaxVals,
                           const QStringList& theSpeedVals,
                           QDomDocument& theDocument)
{
    if (theDimensions.count() < 7) return false;
    if (theAxisX.count() < DL_ROBOT_JOINT_COUNT ||
        theAxisY.count() < DL_ROBOT_JOINT_COUNT ||
        theAxisZ.count() < DL_ROBOT_JOINT_COUNT ||
        theMinVals.count() < DL_ROBOT_JOINT_COUNT ||
        theMaxVals.count() < DL_ROBOT_JOINT_COUNT ||
        theSpeedVals.count() < DL_ROBOT_JOINT_COUNT)
    {
        return false;
    }

    if (!theDocument.setContent(QString::fromUtf8(ROBOT_XML_TEMPLATE))) return false;

    QDomElement aModel = theDocument.documentElement().firstChildElement("model");
    updateTextNode(theDocument, aModel, "manufacturer", theManufacturer);
    updateTextNode(theDocument, aModel, "name", theModelName);

    setFixedTranslation(theDocument, "fixed2", theDimensions[0], "0", theDimensions[1]);
    setFixedTranslation(theDocument, "fixed3", "0", "0", theDimensions[2]);
    setFixedTranslation(theDocument, "fixed4", "0", "0", theDimensions[3]);
    setFixedTranslation(theDocument, "fixed5", theDimensions[4], "0", "0");
    setFixedTranslation(theDocument, "fixed6", "0", theDimensions[5], "0");
    if (theIsPose1)
        setFixedTranslation(theDocument, "fixed7", theDimensions[6], "0", "0");
    else
        setFixedTranslation(theDocument, "fixed7", "0", "0", theDimensions[6]);

    QDomNodeList aJointNodes = theDocument.elementsByTagName("revolute");
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT && i < aJointNodes.count(); ++i)
    {
        QDomElement aJoint = aJointNodes.at(i).toElement();
        QDomElement anAxis = ensureChildElement(theDocument, aJoint, "axis");
        updateTextNode(theDocument, anAxis, "x", theAxisX[i]);
        updateTextNode(theDocument, anAxis, "y", theAxisY[i]);
        updateTextNode(theDocument, anAxis, "z", theAxisZ[i]);
        updateTextNode(theDocument, aJoint, "min", theMinVals[i]);
        updateTextNode(theDocument, aJoint, "max", theMaxVals[i]);
        updateTextNode(theDocument, aJoint, "speed", theSpeedVals[i]);
    }

    return true;
}

// 统一将 Top.xml 中的相对 href 解析为绝对路径。
QString resolveHref(const QString& theBaseDir, const QString& theHref)
{
    if (theHref.isEmpty()) return QString();
    return QFileInfo(QDir(theBaseDir).filePath(theHref)).absoluteFilePath();
}

// 保存 XML 文档时同时负责目录存在性检查。
bool saveXmlDocument(const QDomDocument& theDocument, const QString& theFileName, QString* theErrorMessage = nullptr)
{
    QFileInfo aFileInfo(theFileName);
    if (!QDir().mkpath(aFileInfo.absolutePath()))
    {
        if (nullptr != theErrorMessage)
            *theErrorMessage = QString("Can not create directory: %1").arg(aFileInfo.absolutePath());
        return false;
    }

    QFile aFile(theFileName);
    if (!aFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        if (nullptr != theErrorMessage)
            *theErrorMessage = QString("Can not write file: %1").arg(theFileName);
        return false;
    }

    QTextStream aStream(&aFile);
    aStream.setCodec("UTF-8");
    aStream << theDocument.toString();
    aFile.close();
    return true;
}

// 以法兰局部坐标系为基准，根据 x/y/z(mm 或 m) 与 Rx/Ry/Rz(°) 生成 4x4 刚体变换。
gp_Trsf buildTcpPoseTransform(const double theX,
                             const double theY,
                             const double theZ,
                             const double theRxDeg,
                             const double theRyDeg,
                             const double theRzDeg)
{
    const gp_Pnt anOrigin(0.0, 0.0, 0.0);
    gp_Trsf aTranslation;
    aTranslation.SetTranslation(gp_Vec(theX, theY, theZ));

    gp_Trsf aRotX;
    if (std::abs(theRxDeg) > 0.000001)
        aRotX.SetRotation(gp_Ax1(anOrigin, gp_Dir(1.0, 0.0, 0.0)), theRxDeg * rl::math::DEG2RAD);

    gp_Trsf aRotY;
    if (std::abs(theRyDeg) > 0.000001)
        aRotY.SetRotation(gp_Ax1(anOrigin, gp_Dir(0.0, 1.0, 0.0)), theRyDeg * rl::math::DEG2RAD);

    gp_Trsf aRotZ;
    if (std::abs(theRzDeg) > 0.000001)
        aRotZ.SetRotation(gp_Ax1(anOrigin, gp_Dir(0.0, 0.0, 1.0)), theRzDeg * rl::math::DEG2RAD);

    // 统一采用“先平移，再按 X->Y->Z 输入顺序叠加旋转”的约定。
    gp_Trsf aTransform = aTranslation;
    aTransform.Multiply(aRotZ);
    aTransform.Multiply(aRotY);
    aTransform.Multiply(aRotX);
    return aTransform;
}

// 同一把焊枪在不同 wrist pose 下，STEP 模型局部系到法兰安装系之间存在固定装配修正。
// 这里把该修正单独抽出来，确保 OCC 显示和 RL TCP 计算共用同一套姿态补偿。
gp_Trsf buildToolMountTransform(const bool theIsPose1)
{
    gp_Trsf aTransform;
    if (!theIsPose1)
        return aTransform;

    aTransform.SetRotation(gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 1.0, 0.0)), -90.0 * rl::math::DEG2RAD);
    return aTransform;
}
}

DL_RobotContext::DL_RobotContext(const Handle(AIS_InteractiveContext)& theContext)
: m_context(theContext), m_paramR(0.0), m_paramH(0.0), m_paramL2(0.0), m_paramS(0.0),
  m_paramL3(0.0), m_paramD(0.0), m_paramW(0.0), m_isPose1(true)
{
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) { m_listJoints_angles[i] = 0.0; m_listJoints_angles0[i] = 0.0; }
    for (int i = 0; i <= DL_ROBOT_JOINT_COUNT; ++i)
    {
        m_rodNames[i].clear();
        m_rodFileNames[i].clear();
    }
    m_toolFileName.clear();
    m_isToolEnabled = false;
    m_tfTCPDisplay = gp_Trsf();
    m_tfTCP = gp_Trsf();
    m_tfTCPInv = gp_Trsf();
}

DL_RobotContext::~DL_RobotContext() {}

double* DL_RobotContext::getPositions() { return m_listJoints_angles; }
const double* DL_RobotContext::getPositions() const { return m_listJoints_angles; }
int DL_RobotContext::getRodShapeCount() const { return DL_ROBOT_JOINT_COUNT + 1; }
Handle(AIS_Shape) DL_RobotContext::getRodShape(int theIndex) const { return (theIndex >= 0 && theIndex <= DL_ROBOT_JOINT_COUNT) ? m_listRods[theIndex] : nullptr; }
bool DL_RobotContext::isLoaded() const { return !m_context.IsNull() && m_rl_mdl_KinematicModel && !m_listRods[0].IsNull() && !m_listRods[DL_ROBOT_JOINT_COUNT].IsNull(); }

double DL_RobotContext::getPosition(int theJointIndex) const
{
    return (theJointIndex >= 1 && theJointIndex <= DL_ROBOT_JOINT_COUNT) ? m_listJoints_angles[theJointIndex - 1] : 0.0;
}

void DL_RobotContext::setPositions0(const double* theAngles)
{
    if (!theAngles) return;
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) m_listJoints_angles0[i] = theAngles[i];
}

void DL_RobotContext::setPositions(const double* theAngles)
{
    if (!theAngles) return;
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) m_listJoints_angles[i] = theAngles[i];
    forwardRobot();
    if (!m_context.IsNull()) m_context->UpdateCurrentViewer();
}

void DL_RobotContext::setTcp(const gp_Trsf& theTransform)
{
    m_tfTCP = theTransform;
    m_tfTCPInv = theTransform.Inverted();
}

rl::math::Transform DL_RobotContext::trans(const gp_Trsf& theTransform) const
{
    rl::math::Transform aTransform = rl::math::Transform::Identity();
    for (int r = 0; r < 3; ++r) { for (int c = 0; c < 3; ++c) aTransform(r, c) = theTransform.Value(r + 1, c + 1); aTransform(r, 3) = theTransform.Value(r + 1, 4); }
    return aTransform;
}

gp_Trsf DL_RobotContext::trans(const rl::math::Transform& theTransform) const
{
    gp_Trsf aTransform;
    aTransform.SetValues(theTransform(0, 0), theTransform(0, 1), theTransform(0, 2), theTransform(0, 3),
                         theTransform(1, 0), theTransform(1, 1), theTransform(1, 2), theTransform(1, 3),
                         theTransform(2, 0), theTransform(2, 1), theTransform(2, 2), theTransform(2, 3));
    return aTransform;
}

void DL_RobotContext::loadMdlModel(const char* xmlFileName)
{
    if (!xmlFileName || '\0' == xmlFileName[0]) throw std::runtime_error("robot.xml path is empty");
    m_rl_mdl_IKSolver.reset(); m_rl_mdl_KinematicModel.reset(); m_rl_mdl_Model.reset();
    rl::mdl::XmlFactory aFactory;
    m_rl_mdl_Model.reset(aFactory.create(xmlFileName));
    rl::mdl::Kinematic* aKinematic = dynamic_cast<rl::mdl::Kinematic*>(m_rl_mdl_Model.get());
    if (!aKinematic) { m_rl_mdl_Model.reset(); throw std::runtime_error("RL model is not kinematic"); }
    m_rl_mdl_KinematicModel = std::shared_ptr<rl::mdl::Kinematic>(m_rl_mdl_Model, aKinematic);
    std::printf("RL Kinematics XML loaded successfully once.\n");
}

Handle(AIS_Shape) DL_RobotContext::loadStp(const char* stpFileName)
{
    STEPControl_Reader aReader;
    if (aReader.ReadFile(stpFileName) != IFSelect_RetDone) { std::printf("[Error] 无法读取文件: %s\n", stpFileName); return nullptr; }
    bool showFailsOnly = false;
    std::printf("\n>>> 模型加载检查 [%s]:\n", stpFileName);
    aReader.PrintCheckLoad(showFailsOnly, IFSelect_CountByItem);
    aReader.TransferRoots();
    aReader.PrintCheckTransfer(showFailsOnly, IFSelect_CountByItem);
    TopoDS_Shape aShape = aReader.OneShape();
    if (aShape.IsNull()) { std::printf("[Warning] 文件加载成功但未发现有效几何体\n"); return nullptr; }
    std::printf(">>> 模型加载成功: 找到 %d 个根对象，已合并显示。\n", aReader.NbRootsForTransfer());
    return new AIS_Shape(aShape);
}

Handle(AIS_Shape) DL_RobotContext::loadIges(const char* igesFileName)
{
    IGESControl_Reader aReader;
    if (aReader.ReadFile(igesFileName) != IFSelect_RetDone) return nullptr;
    aReader.TransferRoots();
    TopoDS_Shape aShape = aReader.OneShape();
    return aShape.IsNull() ? nullptr : new AIS_Shape(aShape);
}

bool DL_RobotContext::loadRobotXml()
{
    QFile aFile(m_robotXmlFileName);
    if (!aFile.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
    m_robotXmlDocument.clear();
    bool ok = m_robotXmlDocument.setContent(&aFile);
    aFile.close();
    return ok;
}

double DL_RobotContext::getFixedVal(const char* theId, const char* theAxis) const
{
    QDomNodeList aNodes = m_robotXmlDocument.elementsByTagName("fixed");
    for (int i = 0; i < aNodes.count(); ++i) { QDomElement e = aNodes.at(i).toElement(); if (e.attribute("id") == QString::fromLatin1(theId)) return e.firstChildElement("translation").firstChildElement(QString::fromLatin1(theAxis)).text().toDouble(); }
    return 0.0;
}

gp_Dir DL_RobotContext::getJointDir(int theIndex) const
{
    QDomNodeList aNodes = m_robotXmlDocument.elementsByTagName("revolute");
    if (theIndex < 0 || theIndex >= aNodes.count()) return gp_Dir(0.0, 0.0, 1.0);
    QDomElement aAxis = aNodes.at(theIndex).toElement().firstChildElement("axis");
    double x = aAxis.firstChildElement("x").text().toDouble();
    double y = aAxis.firstChildElement("y").text().toDouble();
    double z = aAxis.firstChildElement("z").text().toDouble();
    return (0.0 == x && 0.0 == y && 0.0 == z) ? gp_Dir(0.0, 0.0, 1.0) : gp_Dir(x, y, z);
}

Handle(AIS_Trihedron) DL_RobotContext::createEndTrihedron(const gp_Pnt& thePoint, const gp_Dir& theNormal, const gp_Dir& theXAxis)
{
    Handle(Geom_Axis2Placement) anAxis = new Geom_Axis2Placement(gp_Ax2(thePoint, theNormal, theXAxis));
    Handle(AIS_Trihedron) aTrihedron = new AIS_Trihedron(anAxis);
    aTrihedron->SetDatumDisplayMode(Prs3d_DM_Shaded);
    aTrihedron->SetArrowColor(Prs3d_DatumParts_XAxis, Quantity_NOC_RED);
    aTrihedron->SetArrowColor(Prs3d_DatumParts_YAxis, Quantity_NOC_GREEN);
    aTrihedron->SetArrowColor(Prs3d_DatumParts_ZAxis, Quantity_NOC_BLUE);
    aTrihedron->SetColor(Quantity_NOC_BLUE);
    aTrihedron->SetXAxisColor(Quantity_NOC_RED);
    aTrihedron->SetYAxisColor(Quantity_NOC_GREEN);
    m_endTrihedron = aTrihedron;
    return aTrihedron;
}

QString DL_RobotContext::findSampleImage(const char* theFileName)
{
    QStringList aDirs;
    aDirs << QDir(QCoreApplication::applicationDirPath()).filePath("samples")
          << QDir(QCoreApplication::applicationDirPath()).filePath("../samples")
          << QDir(QCoreApplication::applicationDirPath()).filePath("../../samples")
          << QDir(QDir::currentPath()).filePath("win/bin/samples")
          << QDir(QDir::currentPath()).filePath("win/bind/samples");
    for (QStringList::const_iterator it = aDirs.constBegin(); it != aDirs.constEnd(); ++it) { QString aPath = QDir(*it).filePath(QString::fromLatin1(theFileName)); if (QFileInfo::exists(aPath)) return QDir::cleanPath(aPath); }
    return QString();
}

void DL_RobotContext::clearRobotPresentation()
{
    if (!m_context.IsNull())
    {
        AIS_ListOfInteractive aObjects;
        m_context->DisplayedObjects(aObjects);
        for (AIS_ListIteratorOfListOfInteractive it(aObjects); it.More(); it.Next())
        {
            if (it.Value()->IsKind(STANDARD_TYPE(AIS_Shape))) m_context->Remove(it.Value(), Standard_False);
        }
        if (!m_endTrihedron.IsNull()) m_context->Remove(m_endTrihedron, Standard_False);
        if (!m_traceLine.IsNull()) m_context->Remove(m_traceLine, Standard_False);
    }

    // 这里既清 OCC 显示，也清当前装载状态；但不主动清空入口 XML 路径，便于下次继续从同目录打开。
    for (int i = 0; i <= DL_ROBOT_JOINT_COUNT; ++i) m_listRods[i].Nullify();
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) m_listJoints_angles[i] = 0.0;
    m_previewStepFileName.clear();
    m_ASTool.Nullify();
    m_traceLine.Nullify();
    m_endTrihedron.Nullify();
    m_rl_mdl_IKSolver.reset();
    m_rl_mdl_KinematicModel.reset();
    m_rl_mdl_Model.reset();
}

void DL_RobotContext::updateTraceLine()
{
    if (m_context.IsNull() || m_endTrihedron.IsNull()) return;
    if (!m_traceLine.IsNull()) m_context->Remove(m_traceLine, Standard_False);
    const gp_Trsf& aTransform = m_endTrihedron->Transformation();
    gp_Pnt aWorldPoint = m_endTrihedron->Component()->Location().Transformed(aTransform);
    TopoDS_Edge anEdge = BRepBuilderAPI_MakeEdge(gp_Pnt(0.0, 0.0, 0.0), aWorldPoint);
    m_traceLine = new AIS_Shape(anEdge);
    m_traceLine->SetColor(Quantity_NOC_GREEN);
    m_traceLine->SetWidth(3.0);
    m_context->Display(m_traceLine, Standard_False);
}

void DL_RobotContext::loadAISShapes()
{
    if (m_robotXmlDocument.isNull()) throw std::runtime_error("robot.xml not loaded");

    // 优先按 Top.xml 中声明的 href 加载几何；缺省时回退到旧的固定命名规则。
    for (int i = 0; i <= DL_ROBOT_JOINT_COUNT; ++i)
    {
        QString aStpPath = m_rodFileNames[i];
        if (aStpPath.isEmpty())
            aStpPath = QDir(m_robotDirPath).filePath(QString("ROD_1_%1.stp").arg(i + 1));
        m_listRods[i] = loadStp(aStpPath.toLocal8Bit().constData());
        if (m_listRods[i].IsNull()) throw std::runtime_error(QString("load rod failed: %1").arg(aStpPath).toStdString());
    }

    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) m_listRods[i]->AddChild(m_listRods[i + 1]);
    for (int i = 0; i <= DL_ROBOT_JOINT_COUNT; ++i) { m_context->SetDisplayMode(m_listRods[i], 1, Standard_False); m_context->Display(m_listRods[i], Standard_False); }

    m_paramR = getFixedVal("fixed2", "x") * 1000.0;
    m_paramH = getFixedVal("fixed2", "z") * 1000.0;
    m_paramL2 = getFixedVal("fixed3", "z") * 1000.0;
    m_paramS = getFixedVal("fixed4", "z") * 1000.0;
    m_paramL3 = getFixedVal("fixed5", "x") * 1000.0;
    m_paramD = getFixedVal("fixed6", "y") * 1000.0;
    double tx7 = getFixedVal("fixed7", "x");
    double tz7 = getFixedVal("fixed7", "z");
    m_isPose1 = std::abs(tx7) > 0.0001;
    m_paramW = (m_isPose1 ? tx7 : tz7) * 1000.0;

    std::printf("\n--- XML 姿态识别结果 ---\n");
    if (m_isPose1) std::printf(">> 检测到 [姿态一：手腕平直], W = %.3f (X轴偏移)\n", tx7);
    else std::printf(">> 检测到 [姿态二：手腕弯折], W = %.3f (Z轴偏移)\n", tz7);

    m_listJoints[0] = gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), getJointDir(0));
    m_listJoints[1] = gp_Ax1(gp_Pnt(m_paramR, 0.0, m_paramH), getJointDir(1));
    m_listJoints[2] = gp_Ax1(gp_Pnt(m_paramR, 0.0, m_paramH + m_paramL2), getJointDir(2));
    m_listJoints[3] = gp_Ax1(gp_Pnt(m_paramR, 0.0, m_paramH + m_paramL2 + m_paramS), getJointDir(3));
    m_listJoints[4] = gp_Ax1(gp_Pnt(m_paramR + m_paramL3, 0.0, m_paramH + m_paramL2 + m_paramS), getJointDir(4));
    if (m_isPose1) m_listJoints[5] = gp_Ax1(gp_Pnt(0.0, m_paramD, m_paramH + m_paramL2 + m_paramS), getJointDir(5));
    else m_listJoints[5] = gp_Ax1(gp_Pnt(m_paramR + m_paramL3, m_paramD, 0.0), getJointDir(5));

    const bool hasToolFile = m_isToolEnabled && !m_toolFileName.isEmpty() && QFileInfo::exists(m_toolFileName);
    gp_Pnt aFlangePoint;
    if (m_isPose1) aFlangePoint.SetCoord(m_paramR + m_paramL3 + m_paramW, m_paramD, m_paramH + m_paramL2 + m_paramS);
    else aFlangePoint.SetCoord(m_paramR + m_paramL3, m_paramD, m_paramH + m_paramL2 + m_paramS + m_paramW);

    gp_Pnt aTriPoint = aFlangePoint;
    gp_Dir aTriNormal(0.0, 0.0, 1.0);
    gp_Dir aTriXAxis(1.0, 0.0, 0.0);
    if (hasToolFile)
    {
        const gp_Pnt aTcpOffset = gp_Pnt(0.0, 0.0, 0.0).Transformed(m_tfTCPDisplay);
        aTriPoint.Translate(gp_Vec(aTcpOffset.X(), aTcpOffset.Y(), aTcpOffset.Z()));
        aTriNormal.Transform(m_tfTCPDisplay);
        aTriXAxis.Transform(m_tfTCPDisplay);
    }

    Handle(AIS_Trihedron) aTrihedron = createEndTrihedron(aTriPoint, aTriNormal, aTriXAxis);
    if (!m_listRods[DL_ROBOT_JOINT_COUNT].IsNull())
    {
        // 与 done/robot.cxx 保持一致：在零位下先把末端坐标系挂到 rod6，
        // 后续只靠父子层级随法兰/焊枪末端运动。
        m_listRods[DL_ROBOT_JOINT_COUNT]->AddChild(aTrihedron);
    }
    m_context->Display(aTrihedron, Standard_False);

    if (hasToolFile)
    {
        gp_Trsf aToolDisplayTransform;
        aToolDisplayTransform.SetTranslation(gp_Vec(aFlangePoint.X(), aFlangePoint.Y(), aFlangePoint.Z()));
        // 工具网格与 TCP 计算共用同一套姿态装配修正，避免“模型装对了但坐标系没跟上”。
        aToolDisplayTransform.Multiply(buildToolMountTransform(m_isPose1));
        loadTool(m_toolFileName.toLocal8Bit().constData(), aToolDisplayTransform);
    }
}

void DL_RobotContext::loadAll()
{
    // 统一走“RL 模型 + OCC 装配 + 初始姿态”三步，避免多入口重复写初始化代码。
    loadMdlModel(m_robotXmlFileName.toLocal8Bit().constData());
    loadAISShapes();
    resetRobot();
}

void DL_RobotContext::forwardRobot()
{
    if (!isLoaded()) return;
    // 同一组关节角同时驱动 OCC 装配层级和 RL 正解模型，保证显示与计算一致。
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i)
    {
        gp_Trsf aTransform;
        aTransform.SetRotation(m_listJoints[i], m_listJoints_angles[i]);
        if (!m_listRods[i + 1].IsNull()) m_listRods[i + 1]->SetLocalTransformation(aTransform);
    }

    std::size_t dof = m_rl_mdl_KinematicModel->getDof();
    rl::math::Vector q(dof);
    for (std::size_t i = 0; i < dof; ++i) q(i) = m_listJoints_angles[i];
    m_rl_mdl_KinematicModel->setPosition(q);
    m_rl_mdl_KinematicModel->forwardPosition();

    updateTraceLine();
}

void DL_RobotContext::resetRobot()
{
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) m_listJoints_angles[i] = m_listJoints_angles0[i];
    forwardRobot();
    if (!m_context.IsNull()) m_context->UpdateCurrentViewer();
}

bool DL_RobotContext::ikSolve(const rl::math::Transform& theTransform, double theAngles[], bool isFromTcp)
{
    if (!isLoaded() || !theAngles) return false;
    rl::math::Transform aGoal = isFromTcp ? theTransform * trans(m_tfTCPInv) : theTransform;
    if (!m_rl_mdl_KinematicModel->calculateInversePosition(aGoal, 0, 0.5)) return false;
    rl::math::Vector q = m_rl_mdl_KinematicModel->getPosition();
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i) theAngles[i] = q(i);
    return true;
}

rl::math::Transform DL_RobotContext::forwardSolve(const double theAngles[], bool isToTcp)
{
    rl::math::Transform aTransform = rl::math::Transform::Identity();
    if (!isLoaded() || !theAngles) return aTransform;
    std::size_t dof = m_rl_mdl_KinematicModel->getDof();
    rl::math::Vector q(dof);
    for (std::size_t i = 0; i < dof; ++i) q(i) = theAngles[i];
    m_rl_mdl_KinematicModel->setPosition(q);
    m_rl_mdl_KinematicModel->forwardPosition();
    aTransform = m_rl_mdl_KinematicModel->getOperationalPosition(0);
    return isToTcp ? aTransform * trans(m_tfTCP) : aTransform;
}

int DL_RobotContext::loadRobotDynamic(QWidget* theParent)
{
    if (m_context.IsNull()) return 0;

    QString aStartPath;
    if (!m_previewStepFileName.isEmpty())
        aStartPath = m_previewStepFileName;
    else if (!m_topXmlFileName.isEmpty())
        aStartPath = m_topXmlFileName;
    else
        aStartPath = m_robotXmlFileName.isEmpty() ? m_robotDirPath : m_robotXmlFileName;
    QString aFileName = QFileDialog::getOpenFileName(
        theParent,
        "Choose Top.xml or STEP file",
        aStartPath,
        "Top XML (*.xml);;STEP Files (*.stp *.step)");
    if (aFileName.isEmpty()) return 0;

    QFileInfo aInfo(aFileName);
    QString aSuffix = aInfo.suffix().toLower();
    if ("xml" == aSuffix)
    {
        // xml 入口专用于 Top.xml 主描述文件。
        return loadRobotFromXml(aFileName, theParent);
    }

    if ("stp" == aSuffix || "step" == aSuffix)
    {
        // stp 入口保留为拆分前总装预览，不进入运动学装载。
        return previewStepFile(aFileName, theParent) ? -1 : 0;
    }

    QMessageBox::warning(theParent, "Tips", "Please choose a Top XML file or a STEP file.");
    return 0;
}

int DL_RobotContext::loadRobotFromXml(const QString& theXmlFileName, QWidget* theParent)
{
    if (m_context.IsNull()) return 0;
    if (theXmlFileName.isEmpty()) return 0;

    QFileInfo aInfo(theXmlFileName);
    QFile aTopFile(aInfo.absoluteFilePath());
    if (!aTopFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QMessageBox::warning(theParent, "Tips", QString("Can not open Top.xml:\n%1").arg(aInfo.absoluteFilePath()));
        return 0;
    }

    QDomDocument aTopDocument;
    if (!aTopDocument.setContent(&aTopFile))
    {
        aTopFile.close();
        QMessageBox::warning(theParent, "Tips", "Top.xml parse failed.");
        return 0;
    }
    aTopFile.close();

    QDomElement aRoot = aTopDocument.documentElement();
    if (aRoot.tagName() != "Top")
    {
        QMessageBox::warning(theParent, "Tips", "Please choose a valid Top.xml file.");
        return 0;
    }

    m_topXmlFileName = aInfo.absoluteFilePath();
    m_robotDirPath = aInfo.absolutePath();
    m_previewStepFileName.clear();
    for (int i = 0; i <= DL_ROBOT_JOINT_COUNT; ++i)
    {
        m_rodNames[i].clear();
        m_rodFileNames[i].clear();
    }

    // Top.xml 负责描述厂商、模型、杆件资源与关键运动学参数。
    QDomElement anInfoElement = aRoot.firstChildElement("Info");
    QString aManufacturer = anInfoElement.attribute("manufacturer", "Huashu");
    QString aModelName = anInfoElement.attribute("model", "HSR-CR630-1750");

    QString aMdlHref("robot.xml");
    QString aToolHref;
    QDomElement aFilesElement = aRoot.firstChildElement("Files");
    if (!aFilesElement.isNull())
    {
        QDomElement aMdlElement = aFilesElement.firstChildElement("Mdl");
        if (!aMdlElement.isNull() && !aMdlElement.attribute("href").isEmpty())
            aMdlHref = aMdlElement.attribute("href");

        QDomElement aToolElement = aFilesElement.firstChildElement("Tool");
        if (!aToolElement.isNull() && !aToolElement.attribute("href").isEmpty())
            aToolHref = aToolElement.attribute("href");
    }
    m_robotXmlFileName = resolveHref(m_robotDirPath, aMdlHref);

    bool aToolEnabled = false;
    gp_Trsf aTcpDisplayTransform;
    gp_Trsf aTcpModelTransform;
    QDomElement aToolConfig = aRoot.firstChildElement("Tool");
    if (!aToolConfig.isNull())
    {
        aToolEnabled = ("false" != aToolConfig.attribute("enabled", "true").toLower());
        QDomElement aTcpElement = aToolConfig.firstChildElement("Tcp");
        const double aTcpX = aTcpElement.attribute("x", "0").toDouble();
        const double aTcpY = aTcpElement.attribute("y", "0").toDouble();
        const double aTcpZ = aTcpElement.attribute("z", "0").toDouble();
        const double aTcpRx = aTcpElement.attribute("rx", "0").toDouble();
        const double aTcpRy = aTcpElement.attribute("ry", "0").toDouble();
        const double aTcpRz = aTcpElement.attribute("rz", "0").toDouble();
        aTcpDisplayTransform = buildTcpPoseTransform(aTcpX, aTcpY, aTcpZ, aTcpRx, aTcpRy, aTcpRz);
        aTcpModelTransform = buildTcpPoseTransform(aTcpX / 1000.0, aTcpY / 1000.0, aTcpZ / 1000.0, aTcpRx, aTcpRy, aTcpRz);
    }
    else if (!aToolHref.isEmpty())
    {
        aToolEnabled = true;
    }

    // Rod 列表只决定几何资源与命名；真正的关节关系仍交给 RL robot.xml。
    QDomElement aRodsElement = aRoot.firstChildElement("Rods");
    for (QDomElement aRod = aRodsElement.firstChildElement("Rod"); !aRod.isNull(); aRod = aRod.nextSiblingElement("Rod"))
    {
        bool isOk = false;
        int anIndex = aRod.attribute("index").toInt(&isOk);
        if (!isOk || anIndex < 0 || anIndex > DL_ROBOT_JOINT_COUNT) continue;

        m_rodNames[anIndex] = aRod.attribute("name", QString::fromLatin1(DEFAULT_ROD_NAMES[anIndex]));
        QString aHref = aRod.attribute("href");
        if (!aHref.isEmpty())
            m_rodFileNames[anIndex] = resolveHref(m_robotDirPath, aHref);
    }

    // Kinematics 是 Top.xml 中对 RL 模板最关键的一层映射来源。
    QDomElement aKinematics = aRoot.firstChildElement("Kinematics");
    QString aPose = aKinematics.attribute("pose", "case1").toLower();
    bool isPose1 = ("case2" != aPose);
    m_isPose1 = isPose1;
    QDomElement aDimensionsElement = aKinematics.firstChildElement("Dimensions");
    QStringList aDimensions;
    aDimensions << aDimensionsElement.attribute("R", "0")
                << aDimensionsElement.attribute("H", "0.278")
                << aDimensionsElement.attribute("L2", "0.380")
                << aDimensionsElement.attribute("S", "0")
                << aDimensionsElement.attribute("L3", "0.370")
                << aDimensionsElement.attribute("D", "-0.125")
                << aDimensionsElement.attribute("W", "-0.135");

    QStringList aAxisX;
    QStringList aAxisY;
    QStringList aAxisZ;
    QStringList aMinVals;
    QStringList aMaxVals;
    QStringList aSpeedVals;
    fillJointDefaults(isPose1, aAxisX, aAxisY, aAxisZ, aMinVals, aMaxVals, aSpeedVals);

    for (QDomElement aJoint = aKinematics.firstChildElement("Joint"); !aJoint.isNull(); aJoint = aJoint.nextSiblingElement("Joint"))
    {
        bool isOk = false;
        int anIndex = aJoint.attribute("index").toInt(&isOk);
        if (!isOk || anIndex < 0 || anIndex >= DL_ROBOT_JOINT_COUNT) continue;

        aAxisX[anIndex] = aJoint.attribute("axisX", aAxisX[anIndex]);
        aAxisY[anIndex] = aJoint.attribute("axisY", aAxisY[anIndex]);
        aAxisZ[anIndex] = aJoint.attribute("axisZ", aAxisZ[anIndex]);
        aMinVals[anIndex] = aJoint.attribute("min", aMinVals[anIndex]);
        aMaxVals[anIndex] = aJoint.attribute("max", aMaxVals[anIndex]);
        aSpeedVals[anIndex] = aJoint.attribute("speed", aSpeedVals[anIndex]);
    }

    // 先构建运行时 robot.xml，再复用既有的 loadAll() 主链，避免改动扩散到后续计算逻辑。
    QDomDocument aRobotDocument;
    if (!buildRobotMdlDocument(aManufacturer,
                               aModelName,
                               isPose1,
                               aDimensions,
                               aAxisX,
                               aAxisY,
                               aAxisZ,
                               aMinVals,
                               aMaxVals,
                               aSpeedVals,
                               aRobotDocument))
    {
        QMessageBox::warning(theParent, "Tips", "Can not build robot.xml from Top.xml.");
        return 0;
    }

    QString anErrorMessage;
    if (!saveXmlDocument(aRobotDocument, m_robotXmlFileName, &anErrorMessage))
    {
        QMessageBox::warning(theParent, "Tips", anErrorMessage);
        return 0;
    }
    m_robotXmlDocument = aRobotDocument;

    try
    {
        clearRobotPresentation();
        const QString aResolvedToolFile = resolveHref(m_robotDirPath, aToolHref);
        const bool hasUsableTool = aToolEnabled && !aResolvedToolFile.isEmpty() && QFileInfo::exists(aResolvedToolFile);
        if (hasUsableTool)
        {
            const gp_Trsf aToolMountTransform = buildToolMountTransform(isPose1);

            gp_Trsf aMountedTcpDisplayTransform = aToolMountTransform;
            aMountedTcpDisplayTransform.Multiply(aTcpDisplayTransform);
            aTcpDisplayTransform = aMountedTcpDisplayTransform;

            gp_Trsf aMountedTcpModelTransform = aToolMountTransform;
            aMountedTcpModelTransform.Multiply(aTcpModelTransform);
            aTcpModelTransform = aMountedTcpModelTransform;
        }

        m_toolFileName = hasUsableTool ? aResolvedToolFile : QString();
        m_isToolEnabled = hasUsableTool;
        m_tfTCPDisplay = hasUsableTool ? aTcpDisplayTransform : gp_Trsf();
        setTcp(hasUsableTool ? aTcpModelTransform : gp_Trsf());
        loadAll();
        if (!m_context.IsNull()) m_context->UpdateCurrentViewer();
        return DL_ROBOT_JOINT_COUNT;
    }
    catch (const std::exception& e)
    {
        clearRobotPresentation();
        std::printf("加载机器人失败: %s\n", e.what());
        QMessageBox::warning(theParent, "Tips", QString("Load robot failed: %1").arg(e.what()));
        return 0;
    }
}

int DL_RobotContext::loadRobot(const QString& theDirPath, QWidget* theParent)
{
    if (m_context.IsNull()) return 0;
    if (theDirPath.isEmpty()) return 0;

    m_topXmlFileName.clear();
    m_robotDirPath = theDirPath;
    m_robotXmlFileName = QDir(theDirPath).filePath("robot.xml");
    m_previewStepFileName.clear();
    m_toolFileName.clear();
    m_isToolEnabled = false;
    m_tfTCPDisplay = gp_Trsf();
    setTcp(gp_Trsf());
    for (int i = 0; i <= DL_ROBOT_JOINT_COUNT; ++i)
    {
        m_rodNames[i].clear();
        m_rodFileNames[i].clear();
    }
    if (!loadRobotXml()) { std::printf("错误：缺失或无法解析 robot.xml 文件。\n"); QMessageBox::warning(theParent, "Tips", "robot.xml missing or invalid."); return 0; }

    try
    {
        clearRobotPresentation();
        loadAll();
        if (!m_context.IsNull()) m_context->UpdateCurrentViewer();
        return DL_ROBOT_JOINT_COUNT;
    }
    catch (const std::exception& e)
    {
        clearRobotPresentation();
        std::printf("加载机器人失败: %s\n", e.what());
        QMessageBox::warning(theParent, "Tips", QString("Load robot failed: %1").arg(e.what()));
        return 0;
    }
}

bool DL_RobotContext::previewStepFile(const QString& theFileName, QWidget* theParent)
{
    if (m_context.IsNull() || theFileName.isEmpty()) return false;

    // 单 STEP 预览故意不初始化 RL 模型，只保留几何查看与后续拆分入口。
    clearRobotPresentation();
    m_toolFileName.clear();
    m_isToolEnabled = false;
    m_tfTCPDisplay = gp_Trsf();
    setTcp(gp_Trsf());
    for (int i = 0; i <= DL_ROBOT_JOINT_COUNT; ++i)
    {
        m_rodNames[i].clear();
        m_rodFileNames[i].clear();
    }

    Handle(AIS_Shape) aPreviewShape = loadStp(theFileName.toLocal8Bit().constData());
    if (aPreviewShape.IsNull())
    {
        QMessageBox::warning(theParent, "Tips", "Load STEP preview failed.");
        return false;
    }

    m_context->SetDisplayMode(aPreviewShape, 1, Standard_False);
    m_context->Display(aPreviewShape, Standard_False);
    m_context->UpdateCurrentViewer();
    m_previewStepFileName = QFileInfo(theFileName).absoluteFilePath();
    m_robotDirPath = QFileInfo(theFileName).absolutePath();
    return true;
}

void DL_RobotContext::moveJoint(int theIndex, int theForward)
{
    if (!isLoaded() || theIndex < 0 || theIndex >= DL_ROBOT_JOINT_COUNT) return;
    m_listJoints_angles[theIndex] += theForward * 5.0 * rl::math::DEG2RAD;
    forwardRobot();
    if (!m_context.IsNull()) m_context->UpdateCurrentViewer();
}

void DL_RobotContext::calcRobot()
{
    if (!isLoaded() || m_endTrihedron.IsNull()) return;
    std::size_t dof = m_rl_mdl_KinematicModel->getDof();

    // 这里主要用于验证“当前显示姿态”与“RL 正逆解结果”是否保持一致。
    std::cout << "================ 关节限位（单位：度） ================" << std::endl;
    for (std::size_t i = 0; i < dof; ++i)
    {
        const rl::mdl::Joint* aJoint = m_rl_mdl_KinematicModel->getJoint(i);
        std::cout << "Joint " << i << ": min = " << aJoint->min(0) * rl::math::RAD2DEG << " °, max = " << aJoint->max(0) * rl::math::RAD2DEG << " °" << std::endl;
    }
    std::cout << "=====================================================" << std::endl;

    rl::math::Vector q(dof), q0(dof), qinv(dof);
    for (std::size_t i = 0; i < dof; ++i) { q(i) = m_listJoints_angles[i]; q0(i) = m_listJoints_angles0[i]; }
    const bool isUsingToolTcp = !m_ASTool.IsNull();
    rl::math::Transform t = forwardSolve(m_listJoints_angles, isUsingToolTcp);
    std::cout << "rl依据xml计算末端坐标 (X,Y,Z): " << t.translation().x() << ", " << t.translation().y() << ", " << t.translation().z() << std::endl;

    gp_Pnt aWorldPoint = m_endTrihedron->Component()->Location().Transformed(m_endTrihedron->Transformation());
    std::cout << "OCC 真实末端坐标 (X,Y,Z): " << aWorldPoint.X() << ", " << aWorldPoint.Y() << ", " << aWorldPoint.Z() << std::endl;

    m_rl_mdl_KinematicModel->setPosition(q0);
    m_rl_mdl_KinematicModel->forwardPosition();

    double aAngles[DL_ROBOT_JOINT_COUNT] = {0.0};
    if (!ikSolve(t, aAngles, isUsingToolTcp)) { std::printf("错误：逆解计算失败，无法到达目标位姿。\n"); return; }

    for (std::size_t i = 0; i < dof; ++i) { m_listJoints_angles[i] = aAngles[i]; qinv(i) = aAngles[i]; }
    forwardRobot();
    if (!m_context.IsNull()) m_context->UpdateCurrentViewer();

    rl::math::Transform tinv = forwardSolve(m_listJoints_angles, isUsingToolTcp);
    std::cout << "================ 正逆解一致性验证 ================" << std::endl;
    std::cout << "初始目标关节角 (q):    " << q.transpose() << std::endl;
    std::cout << "逆解计算关节角 (qinv): " << qinv.transpose() << std::endl;
    std::cout << "\n正解矩阵 (t):\n" << t.matrix() << std::endl;
    std::cout << "\n逆解矩阵 (tinv):\n" << tinv.matrix() << std::endl;
}

void DL_RobotContext::disasRobot(QWidget* theParent)
{
    if (m_context.IsNull()) return;
    std::printf("--- 开始执行拆分流程 ---\n");

    QString aFileName = m_previewStepFileName;
    if (aFileName.isEmpty() || !QFileInfo::exists(aFileName))
    {
        QString aStartPath = !m_previewStepFileName.isEmpty() ? m_previewStepFileName : m_robotDirPath;
        aFileName = QFileDialog::getOpenFileName(theParent, "choose stp or step", aStartPath, "STEP Files (*.stp *.step)");
    }
    if (aFileName.isEmpty()) return;
    splitStepFile(aFileName);
}

bool DL_RobotContext::splitStepFile(const QString& theFileName)
{
    if (m_context.IsNull() || theFileName.isEmpty()) return false;

    QString aFileName = theFileName;
    QString aSaveDir = QFileInfo(aFileName).absolutePath();
    STEPControl_Reader aReader;
    if (aReader.ReadFile(aFileName.toLocal8Bit().constData()) != IFSelect_RetDone) { std::printf("读取失败\n"); return false; }
    aReader.TransferRoots();

    // 保持原有拆分策略：按 STEP 顶层 root 的直接子件拆分，并继续使用爆炸视图显示结果。
    TopoDS_Compound aWholeCompound;
    BRep_Builder aBuilder;
    aBuilder.MakeCompound(aWholeCompound);

    int aShapesNumber = aReader.NbShapes();
    Standard_Real anOffset = 500.0;
    for (int i = 1; i <= aShapesNumber; ++i)
    {
        TopoDS_Shape aRootShape = aReader.Shape(i);
        int aPartIndex = 0;
        for (TopoDS_Iterator it(aRootShape); it.More(); it.Next())
        {
            TopoDS_Shape aPart = it.Value();
            ++aPartIndex;
            QString anOutputName = QString("%1/ROD_%2_%3.stp").arg(aSaveDir).arg(i).arg(aPartIndex);

            STEPControl_Writer aWriter;
            aWriter.Transfer(aPart, STEPControl_AsIs);
            aWriter.Write(anOutputName.toLocal8Bit().constData());
            std::printf("零件已保存至: %s\n", anOutputName.toLocal8Bit().constData());

            Handle(AIS_Shape) anAisPart = new AIS_Shape(aPart);
            gp_Trsf aTransform;
            aTransform.SetTranslation(gp_Vec(0.0, anOffset, 0.0));
            anAisPart->SetLocalTransformation(aTransform);
            m_context->SetDisplayMode(anAisPart, 1, Standard_False);
            m_context->Display(anAisPart, Standard_False);
            anOffset += 500.0;
            aBuilder.Add(aWholeCompound, aPart);
        }
    }

    Handle(AIS_Shape) anAisWhole = new AIS_Shape(aWholeCompound);
    m_context->SetDisplayMode(anAisWhole, 1, Standard_False);
    m_context->Display(anAisWhole, Standard_True);
    std::printf("--- 拆分显示完成 ---\n");
    return true;
}

void DL_RobotContext::loadTool(const char* modelFileName, const gp_Trsf& theDisplayTransform)
{
    if (!modelFileName) return;
    QString aFileName = QString::fromLocal8Bit(modelFileName);
    if (!QFileInfo::exists(aFileName)) return;
    QString aSuffix = QFileInfo(aFileName).suffix().toLower();
    m_ASTool = ("igs" == aSuffix || "iges" == aSuffix) ? loadIges(modelFileName) : loadStp(modelFileName);
    if (m_ASTool.IsNull()) return;
    m_ASTool->SetLocalTransformation(theDisplayTransform);
    if (!m_listRods[DL_ROBOT_JOINT_COUNT].IsNull()) m_listRods[DL_ROBOT_JOINT_COUNT]->AddChildWithCurrentTransformation(m_ASTool);
    if (!m_context.IsNull()) { m_context->SetDisplayMode(m_ASTool, 1, Standard_False); m_context->Display(m_ASTool, Standard_False); m_context->UpdateCurrentViewer(); }
}

void DL_RobotContext::writeRobotXml(QWidget* theParent)
{
    QDialog aDialog(theParent);
    aDialog.setWindowTitle("Write Robot Xml");
    QHBoxLayout* aMainLayout = new QHBoxLayout(&aDialog);

    QVBoxLayout* aLeftLayout = new QVBoxLayout();
    QLabel* aImageLabel = new QLabel();
    QString aPose1Image = findSampleImage("robotXml.png");
    QString aPose2Image = findSampleImage("down.png");
    auto updateImage = [aImageLabel, aPose1Image, aPose2Image](bool isPose1)
    {
        QString aPath = isPose1 ? aPose1Image : aPose2Image;
        if (aPath.isEmpty()) aImageLabel->clear();
        else aImageLabel->setPixmap(QPixmap(aPath).scaled(350, 450, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    };

    QRadioButton* aPose1Button = new QRadioButton("Case1:Straight (W in X)");
    QRadioButton* aPose2Button = new QRadioButton("Case2:Bend (W in Z)");
    aPose1Button->setChecked(true);
    updateImage(true);
    QObject::connect(aPose1Button, &QRadioButton::toggled, [&updateImage](bool checked) { updateImage(checked); });

    aLeftLayout->addWidget(aImageLabel);
    aLeftLayout->addWidget(aPose1Button);
    aLeftLayout->addWidget(aPose2Button);
    aLeftLayout->addStretch();
    aMainLayout->addLayout(aLeftLayout);

    QGridLayout* aGrid = new QGridLayout();
    aGrid->setHorizontalSpacing(15);
    aGrid->setVerticalSpacing(10);
    QLineEdit* aManufacturerEdit = new QLineEdit("Huashu");
    QLineEdit* aModelEdit = new QLineEdit("HSR-CR630-1750");
    aManufacturerEdit->setAlignment(Qt::AlignCenter);
    aModelEdit->setAlignment(Qt::AlignCenter);
    QLabel* aManufacturerLabel = new QLabel("Manufacturer");
    QLabel* aModelLabel = new QLabel("Model");
    aManufacturerLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    aModelLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    aGrid->addWidget(aManufacturerLabel, 0, 0);
    aGrid->addWidget(aManufacturerEdit, 0, 1, 1, 6);
    aGrid->addWidget(aModelLabel, 1, 0);
    aGrid->addWidget(aModelEdit, 1, 1, 1, 6);

    QLineEdit* aTcpEdits[6] = {nullptr};
    QStringList aTcpLabels = QStringList() << "Tcp X(mm)" << "Tcp Y(mm)" << "Tcp Z(mm)"
                                           << "Tcp Rx(Degree)" << "Tcp Ry(Degree)" << "Tcp Rz(Degree)";
    for (int i = 0; i < 6; ++i)
    {
        const int aRow = 2 + i / 3;
        const int aColumn = (i % 3) * 2;
        QLabel* aTcpLabel = new QLabel(aTcpLabels[i]);
        aTcpLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        aGrid->addWidget(aTcpLabel, aRow, aColumn);
        aTcpEdits[i] = new QLineEdit("0");
        aTcpEdits[i]->setAlignment(Qt::AlignCenter);
        aGrid->addWidget(aTcpEdits[i], aRow, aColumn + 1);
    }

    QLineEdit* aParamEdits[7] = {nullptr};
    QStringList aParamLabels = QStringList() << "R" << "H" << "L2" << "S" << "L3" << "D" << "W";
    QStringList aParamDefaults = QStringList() << "0" << "0.278" << "0.380" << "0" << "0.370" << "-0.125" << "-0.135";
    for (int i = 0; i < aParamLabels.count(); ++i)
    {
        QLabel* aLabel = new QLabel(aParamLabels[i]);
        aLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        aGrid->addWidget(aLabel, i + 4, 0);
        aParamEdits[i] = new QLineEdit(aParamDefaults[i]);
        aParamEdits[i]->setAlignment(Qt::AlignCenter);
        aGrid->addWidget(aParamEdits[i], i + 4, 1);
    }
    aGrid->setColumnMinimumWidth(2, 20);

    QLabel* aMinTitle = new QLabel("Min(Degree)");
    QLabel* aMaxTitle = new QLabel("Max(Degree)");
    QLabel* aSpeedTitle = new QLabel("MaxSpeed");
    aMinTitle->setAlignment(Qt::AlignCenter);
    aMaxTitle->setAlignment(Qt::AlignCenter);
    aSpeedTitle->setAlignment(Qt::AlignCenter);
    aGrid->addWidget(aMinTitle, 4, 4);
    aGrid->addWidget(aMaxTitle, 4, 5);
    aGrid->addWidget(aSpeedTitle, 4, 6);

    QLineEdit* aMinEdits[DL_ROBOT_JOINT_COUNT] = {nullptr};
    QLineEdit* aMaxEdits[DL_ROBOT_JOINT_COUNT] = {nullptr};
    QLineEdit* aSpeedEdits[DL_ROBOT_JOINT_COUNT] = {nullptr};
    QStringList aSpeedDefaults = QStringList() << "187" << "160" << "180" << "260" << "230" << "360";
    for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i)
    {
        QLabel* aJointLabel = new QLabel(QString("Joint %1:").arg(i));
        aJointLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        aGrid->addWidget(aJointLabel, i + 5, 3);

        aMinEdits[i] = new QLineEdit("-360");
        aMaxEdits[i] = new QLineEdit("360");
        aSpeedEdits[i] = new QLineEdit(aSpeedDefaults[i]);
        aMinEdits[i]->setAlignment(Qt::AlignCenter);
        aMaxEdits[i]->setAlignment(Qt::AlignCenter);
        aSpeedEdits[i]->setAlignment(Qt::AlignCenter);
        aGrid->addWidget(aMinEdits[i], i + 5, 4);
        aGrid->addWidget(aMaxEdits[i], i + 5, 5);
        aGrid->addWidget(aSpeedEdits[i], i + 5, 6);
    }

    QPushButton* aCreateButton = new QPushButton("Create Top.xml + robot.xml");
    aGrid->addWidget(aCreateButton, 11, 0, 1, 7);
    aMainLayout->addLayout(aGrid);

    QObject::connect(aCreateButton, &QPushButton::clicked, [&]()
    {
        // 表单只维护一份输入，再同时输出 Top.xml 与 robot.xml，避免两份配置手工同步。
        bool isPose1 = aPose1Button->isChecked();
        QString aSavePath = QFileDialog::getExistingDirectory(&aDialog, "Save to...");
        if (aSavePath.isEmpty()) return;

        QStringList aDimensions;
        for (int i = 0; i < aParamLabels.count(); ++i) aDimensions << aParamEdits[i]->text();
        QStringList aTcpValues;
        for (int i = 0; i < 6; ++i) aTcpValues << aTcpEdits[i]->text();

        QStringList aAxisX;
        QStringList aAxisY;
        QStringList aAxisZ;
        QStringList aMinVals;
        QStringList aMaxVals;
        QStringList aSpeedVals;
        fillJointDefaults(isPose1, aAxisX, aAxisY, aAxisZ, aMinVals, aMaxVals, aSpeedVals);
        for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i)
        {
            aMinVals[i] = aMinEdits[i]->text();
            aMaxVals[i] = aMaxEdits[i]->text();
            aSpeedVals[i] = aSpeedEdits[i]->text();
        }

        QDomDocument aRobotDocument;
        if (!buildRobotMdlDocument(aManufacturerEdit->text(),
                                   aModelEdit->text(),
                                   isPose1,
                                   aDimensions,
                                   aAxisX,
                                   aAxisY,
                                   aAxisZ,
                                   aMinVals,
                                   aMaxVals,
                                   aSpeedVals,
                                   aRobotDocument))
        {
            QMessageBox::warning(&aDialog, "Tips", "robot.xml build failed.");
            return;
        }

        QDomDocument aTopDocument;
        QDomElement aTopRoot = aTopDocument.createElement("Top");
        aTopRoot.setAttribute("version", "1.0");
        aTopDocument.appendChild(aTopRoot);

        QDomElement anInfo = aTopDocument.createElement("Info");
        anInfo.setAttribute("manufacturer", aManufacturerEdit->text());
        anInfo.setAttribute("model", aModelEdit->text());
        anInfo.setAttribute("dof", QString::number(DL_ROBOT_JOINT_COUNT));
        aTopRoot.appendChild(anInfo);

        QDomElement aFiles = aTopDocument.createElement("Files");
        QDomElement aMdl = aTopDocument.createElement("Mdl");
        aMdl.setAttribute("href", "robot.xml");
        aFiles.appendChild(aMdl);
        QDomElement aSourceStep = aTopDocument.createElement("SourceStep");
        aSourceStep.setAttribute("href", "");
        aFiles.appendChild(aSourceStep);
        QDomElement aToolFile = aTopDocument.createElement("Tool");
        aToolFile.setAttribute("href", "welding gun.stp");
        aFiles.appendChild(aToolFile);
        aTopRoot.appendChild(aFiles);

        // 当前 UI 没有单独编辑杆件名称与 href，因此先按既有固定命名写出默认资源清单。
        QDomElement aRods = aTopDocument.createElement("Rods");
        for (int i = 0; i <= DL_ROBOT_JOINT_COUNT; ++i)
        {
            QDomElement aRod = aTopDocument.createElement("Rod");
            aRod.setAttribute("index", QString::number(i));
            aRod.setAttribute("name", QString::fromLatin1(DEFAULT_ROD_NAMES[i]));
            aRod.setAttribute("body", QString("body%1").arg(i));
            aRod.setAttribute("href", QString("ROD_1_%1.stp").arg(i + 1));
            aRods.appendChild(aRod);
        }
        aTopRoot.appendChild(aRods);

        QDomElement aKinematics = aTopDocument.createElement("Kinematics");
        aKinematics.setAttribute("unit", "m");
        aKinematics.setAttribute("pose", isPose1 ? "case1" : "case2");
        QDomElement aDimensionsElement = aTopDocument.createElement("Dimensions");
        aDimensionsElement.setAttribute("R", aDimensions[0]);
        aDimensionsElement.setAttribute("H", aDimensions[1]);
        aDimensionsElement.setAttribute("L2", aDimensions[2]);
        aDimensionsElement.setAttribute("S", aDimensions[3]);
        aDimensionsElement.setAttribute("L3", aDimensions[4]);
        aDimensionsElement.setAttribute("D", aDimensions[5]);
        aDimensionsElement.setAttribute("W", aDimensions[6]);
        aKinematics.appendChild(aDimensionsElement);

        for (int i = 0; i < DL_ROBOT_JOINT_COUNT; ++i)
        {
            QDomElement aJoint = aTopDocument.createElement("Joint");
            aJoint.setAttribute("index", QString::number(i));
            aJoint.setAttribute("name", QString("J%1").arg(i + 1));
            aJoint.setAttribute("axisX", aAxisX[i]);
            aJoint.setAttribute("axisY", aAxisY[i]);
            aJoint.setAttribute("axisZ", aAxisZ[i]);
            aJoint.setAttribute("min", aMinVals[i]);
            aJoint.setAttribute("max", aMaxVals[i]);
            aJoint.setAttribute("speed", aSpeedVals[i]);
            aKinematics.appendChild(aJoint);
        }
        aTopRoot.appendChild(aKinematics);

        QDomElement aToolElement = aTopDocument.createElement("Tool");
        aToolElement.setAttribute("enabled", "true");
        QDomElement aTcpElement = aTopDocument.createElement("Tcp");
        aTcpElement.setAttribute("x", aTcpValues[0]);
        aTcpElement.setAttribute("y", aTcpValues[1]);
        aTcpElement.setAttribute("z", aTcpValues[2]);
        aTcpElement.setAttribute("rx", aTcpValues[3]);
        aTcpElement.setAttribute("ry", aTcpValues[4]);
        aTcpElement.setAttribute("rz", aTcpValues[5]);
        aToolElement.appendChild(aTcpElement);
        aTopRoot.appendChild(aToolElement);

        QString anErrorMessage;
        if (!saveXmlDocument(aRobotDocument, QDir(aSavePath).filePath("robot.xml"), &anErrorMessage))
        {
            QMessageBox::warning(&aDialog, "Tips", anErrorMessage);
            return;
        }
        if (!saveXmlDocument(aTopDocument, QDir(aSavePath).filePath("Top.xml"), &anErrorMessage))
        {
            QMessageBox::warning(&aDialog, "Tips", anErrorMessage);
            return;
        }

        QMessageBox::information(&aDialog, "Done", "Top.xml and robot.xml created!");
        aDialog.accept();
    });

    aDialog.exec();
}
