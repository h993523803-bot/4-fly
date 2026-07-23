#ifndef __MENU_H
#define __MENU_H

#define MAX_NAME_LENGTH			32
#define MAX_MENU_LIST_LENGTH	32

struct Menu_Item {
	char Name[MAX_NAME_LENGTH];
	void (*Function)(void);
	struct Menu_List *NextList;
};

struct Menu_List {
	char Name[MAX_NAME_LENGTH];
	void (*BackFunction)(void);
	int16_t WindowIndex;
	int16_t WindowLength;
	int16_t ItemIndex;
	int16_t ItemLength;
	struct Menu_Item Items[32];
};

extern struct Menu_List MainMenu;
extern struct Menu_List HardwareTestMenu;
extern struct Menu_List CalibrationMenu;
extern struct Menu_List BluetoothMenu;

extern struct Menu_List *ListStack[];

void Menu_Display(void);

void Menu_Down(void);
void Menu_Up(void);
void Menu_Enter(void);
void Menu_Back(void);

void Menu_Init(void);
void Menu_Loop(void);
void Menu_Exit(void);

#endif
