#ifndef _DEMO_MENU_H
#define _DEMO_MENU_H

class NewtonDemos;
class DemoEntityManager;


typedef void (*LaunchSDKDemoCallback) (DemoEntityManager* scene);

class SDKDemos
{
	public:
	const char *m_name;
	const char *m_description;
	LaunchSDKDemoCallback m_launchDemoCallback;
};


class DemoMenu: public FXMenuBar 
{
	public:
	DemoMenu(FXComposite* const parent, NewtonDemos* const mainFrame);
	~DemoMenu(void);

	void LoadDemo (DemoEntityManager* const scene, int index);

	FXMenuPane* m_fileMenu;
	FXMenuPane* m_optionsMenu;
	FXMenuPane* m_helpMenu;
	FXMenuPane* m_profilerSubMenu;
	FXMenuPane* m_microThreadedsSubMenu;
	FXMenuCheck* m_profilerTracksMenu[8];
	FXMenuRadio* m_cpuModes[3];
	int m_threadsTracks[7];
};


#endif