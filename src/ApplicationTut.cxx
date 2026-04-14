#include "ApplicationTut.h"
#include "DL_RobotContext.h"
#include "DocumentTut.h"

#include <OSD_Environment.hxx>

#include <Standard_WarningsDisable.hxx>
#include <QFileDialog>
#include <QStatusBar>
#include <QMdiSubWindow>
#include <Standard_WarningsRestore.hxx>

namespace
{
DocumentTut* documentFromSubWindow(QMdiSubWindow* theSubWindow)
{
  if (theSubWindow == NULL)
  {
    return NULL;
  }

  MDIWindow* aWindow = qobject_cast<MDIWindow*>(theSubWindow->widget());
  if (aWindow == NULL)
  {
    return NULL;
  }

  return qobject_cast<DocumentTut*>(aWindow->getDocument());
}

DocumentTut* activeDocument(QMdiArea* theWorkspace)
{
  if (theWorkspace == NULL)
  {
    return NULL;
  }

  if (DocumentTut* aDocument = documentFromSubWindow(theWorkspace->activeSubWindow()))
  {
    return aDocument;
  }

  QList<QMdiSubWindow*> aWindows = theWorkspace->subWindowList();
  for (int anIndex = 0; anIndex < aWindows.count(); ++anIndex)
  {
    DocumentTut* aDocument = documentFromSubWindow(aWindows.at(anIndex));
    if (aDocument != NULL)
    {
      theWorkspace->setActiveSubWindow(aWindows.at(anIndex));
      return aDocument;
    }
  }

  return NULL;
}
}

ApplicationTut::ApplicationTut()
    : ApplicationCommonWindow( ),
      myLoadRobotAction(NULL),
      mySplitRobotAction(NULL),
      myWriteRobotAction(NULL),
      myJointSelectAction(NULL),
      myJointForwardAction(NULL),
      myJointBackwardAction(NULL),
      myRobotResetAction(NULL),
      myRobotCalcAction(NULL),
      nCurrentJoint_Index(0),
      nMaxJoint_Count(0)
{
  createMakeBottleOperation();
  updateRobotActionStates();
}

ApplicationTut::~ApplicationTut()
{
}

DocumentCommon* ApplicationTut::createNewDocument()
{
  return new DocumentTut(++myNbDocuments, this);
}

#define ROBOT_ACTION(ICON_ID,NAME,TIP,STIP,_SLOT)\
	toolbarIcon = QPixmap( dir+QObject::tr( #ICON_ID ) );\
	MakeBottleAction = new QAction( toolbarIcon, QObject::tr(#NAME), this );\
	MakeBottleAction->setToolTip( QObject::tr( #TIP ) );\
	MakeBottleAction->setStatusTip( QObject::tr(#STIP) );\
	connect( MakeBottleAction, SIGNAL( triggered() ) , this, SLOT( _SLOT ) );\
	myMakeBottleBar->addAction( MakeBottleAction );
	
#define ROBOT_ACTIONX(ID,_SLOT)\
	ROBOT_ACTION(ICON_##ID,ID,TBR_##ID,INF_##ID,_SLOT)


void ApplicationTut::createMakeBottleOperation()
{
	QString dir = getTutResourceDir() + QString( "/" );
	QPixmap toolbarIcon;

	myMakeBottleBar = addToolBar( tr( "Make Bottle" ) );
	QAction * MakeBottleAction;
	
    ROBOT_ACTIONX(ROBOT_DISAS,onRobotDisas())
    mySplitRobotAction = MakeBottleAction;
    MakeBottleAction->setShortcut(QString( "CTRL+D" ) ); //新增拆分按钮

    ROBOT_ACTIONX(ROBOT_WRITE,onRobotWrite())
    myWriteRobotAction = MakeBottleAction;
    MakeBottleAction->setShortcut(QString( "CTRL+W" ) ); //新增编写xml按钮

	ROBOT_ACTIONX(MAKE_BOTTLE,onMakeBottleAction())
    myLoadRobotAction = MakeBottleAction;
	MakeBottleAction->setShortcut(QString( "CTRL+M" ) );
	
	ROBOT_ACTIONX(JOINT_SELECT,onJointSelect())
    myJointSelectAction = MakeBottleAction;
	MakeBottleAction->setShortcut(QString( "CTRL+S" ) );
	
	ROBOT_ACTIONX(JOINT_FWORD,onJointForward())
    myJointForwardAction = MakeBottleAction;
	MakeBottleAction->setShortcut(QString( "CTRL+F" ) );
	
	ROBOT_ACTIONX(JOINT_BWORD,onJointBckward())
    myJointBackwardAction = MakeBottleAction;
	MakeBottleAction->setShortcut(QString( "CTRL+B" ) );

	ROBOT_ACTIONX(ROBOT_RESET,onRobotReset())
    myRobotResetAction = MakeBottleAction;
	MakeBottleAction->setShortcut(QString( "CTRL+R" ) );

    ROBOT_ACTIONX(ROBOT_CALC,onRobotCalc())
    myRobotCalcAction = MakeBottleAction;
    MakeBottleAction->setShortcut(QString( "CTRL+C" ) ); //新增计算按钮

	myMakeBottleBar->hide();
	insertToolBar( getCasCadeBar(), myMakeBottleBar );
}

void ApplicationTut::updateFileActions()
{
  if ( getWorkspace()->subWindowList().isEmpty() )
  {
	  if ( !isDocument() )
		{
      myMakeBottleBar->show();
    }
    else
    {
      myMakeBottleBar->hide();
    }
  }
  ApplicationCommonWindow::updateFileActions();
  updateRobotActionStates();
}

void ApplicationTut::updateRobotActionStates()
{
  DocumentTut* aDocument = activeDocument(getWorkspace());
  const bool hasDocument = aDocument != NULL;

  if (myLoadRobotAction != NULL) myLoadRobotAction->setEnabled(hasDocument);
  if (mySplitRobotAction != NULL) mySplitRobotAction->setEnabled(hasDocument);
  if (myWriteRobotAction != NULL) myWriteRobotAction->setEnabled(hasDocument);
  if (myJointSelectAction != NULL) myJointSelectAction->setEnabled(hasDocument);
  if (myJointForwardAction != NULL) myJointForwardAction->setEnabled(hasDocument);
  if (myJointBackwardAction != NULL) myJointBackwardAction->setEnabled(hasDocument);
  if (myRobotResetAction != NULL) myRobotResetAction->setEnabled(hasDocument);
  if (myRobotCalcAction != NULL) myRobotCalcAction->setEnabled(hasDocument);

  if (!hasDocument)
  {
    nCurrentJoint_Index = 0;
    nMaxJoint_Count = 0;
  }
}

void ApplicationTut::onMakeBottleAction()
{
	DocumentTut* doc = activeDocument(ApplicationCommonWindow::getWorkspace());
	if (doc == NULL)
	{
		statusBar()->showMessage(QObject::tr("Please open a occ3D Window!"), 5000);
		return;
	}
  	
	statusBar()->showMessage( QObject::tr("INF_MAKE_BOTTLE"), 5000 );
	
	nMaxJoint_Count = doc->onMakeBottle();
	nCurrentJoint_Index = 0;
    updateRobotActionStates();
	
	if (doc->getRobot()->canDrive() && nMaxJoint_Count > 0)
	{
		statusBar()->showMessage(QString("Current Joint: J#")+QString::number(nCurrentJoint_Index+1)+" of "+QString::number(nMaxJoint_Count));
	}
    else if (doc->getRobot()->loadMode() == DL_RobotContext::LoadMode_AssemblyPreview)
    {
        statusBar()->showMessage(QString::fromLocal8Bit("已载入整机 STP 预览，可继续执行拆分。"), 5000);
    }
    else if (doc->getRobot()->loadMode() == DL_RobotContext::LoadMode_RobotPackage)
    {
        statusBar()->showMessage(QString::fromLocal8Bit("已载入机械臂模型，可继续进行关节和计算操作。"), 5000);
    }
}


QString ApplicationTut::getTutResourceDir()
{
  static QString resDir (OSD_Environment ("CSF_TutorialResourcesDefaults").Value().ToCString());
  if (resDir.isEmpty())
    resDir = QString (OSD_Environment ("CSF_OCCTResourcePath").Value().ToCString()) + "./samples";
  return resDir;
}
