#include "ApplicationTut.h"
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
      nCurrentJoint_Index(0),
      nMaxJoint_Count(0)
{
  createMakeBottleOperation();
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
    MakeBottleAction->setShortcut(QString( "CTRL+D" ) ); //新增拆分按钮

    ROBOT_ACTIONX(ROBOT_WRITE,onRobotWrite())
    MakeBottleAction->setShortcut(QString( "CTRL+W" ) ); //新增编写xml按钮

    ROBOT_ACTIONX(MAKE_BOTTLE,onMakeBottleAction())
    MakeBottleAction->setShortcut(QString( "CTRL+M" ) );

    ROBOT_ACTIONX(JOINT_SELECT,onJointSelect())
    MakeBottleAction->setShortcut(QString( "CTRL+S" ) );

    ROBOT_ACTIONX(JOINT_FWORD,onJointForward())
    MakeBottleAction->setShortcut(QString( "CTRL+F" ) );

    ROBOT_ACTIONX(JOINT_BWORD,onJointBckward())
    MakeBottleAction->setShortcut(QString( "CTRL+B" ) );

    ROBOT_ACTIONX(ROBOT_RESET,onRobotReset())
    MakeBottleAction->setShortcut(QString( "CTRL+R" ) );

    ROBOT_ACTIONX(ROBOT_CALC,onRobotCalc())
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

    if (nMaxJoint_Count > 0)
    {
        statusBar()->showMessage(QString("Current Joint: J#")+QString::number(nCurrentJoint_Index+1)+" of "+QString::number(nMaxJoint_Count));
    }
}


QString ApplicationTut::getTutResourceDir()
{
  static QString resDir (OSD_Environment ("CSF_TutorialResourcesDefaults").Value().ToCString());
  if (resDir.isEmpty())
    resDir = QString (OSD_Environment ("CSF_OCCTResourcePath").Value().ToCString()) + "./samples";
  return resDir;
}
