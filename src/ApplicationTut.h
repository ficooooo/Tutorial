#ifndef APPLICATIONTUT_H
#define APPLICATIONTUT_H

#include "DocumentTut.h"
#include "ApplicationCommon.h"

class QAction;

class ApplicationTut: public ApplicationCommonWindow
{
    Q_OBJECT
public:
  ApplicationTut();
  ~ApplicationTut();
  static QString    getTutResourceDir();
  virtual void      updateFileActions();
protected:
  virtual DocumentCommon* createNewDocument();
public slots:
	void			onMakeBottleAction();
	void			onJointSelect();
	void			onJointForward();
	void			onJointBckward();
	void			onRobotReset();
    void            onRobotCalc();
    void            onRobotDisas();
    void            onRobotWrite();
private:
	void            createMakeBottleOperation();
    void            updateRobotActionStates();
private:
	QToolBar*		myMakeBottleBar;
    QAction*        myLoadRobotAction;
    QAction*        mySplitRobotAction;
    QAction*        myWriteRobotAction;
    QAction*        myJointSelectAction;
    QAction*        myJointForwardAction;
    QAction*        myJointBackwardAction;
    QAction*        myRobotResetAction;
    QAction*        myRobotCalcAction;
	int				nCurrentJoint_Index;
	int				nMaxJoint_Count;
};

#endif
