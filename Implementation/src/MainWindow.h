#pragma once
#include <gui/Window.h>
#include "MainView.h"

class MainWindow : public gui::Window
{
protected:
    MainView _mainView;

protected:
    void onInitialAppearance() override
    {
        _mainView.focusOnCanvas();
    }

public:
    MainWindow()
        : gui::Window(gui::Size(1200, 800))
        , _mainView()
    {
        setTitle("WiFi AP Placement Optimization");
        setCentralView(&_mainView);
    }
};