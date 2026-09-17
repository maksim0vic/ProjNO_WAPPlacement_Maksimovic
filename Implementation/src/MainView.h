#pragma once
#include <gui/View.h>
#include <gui/GridLayout.h>
#include <gui/GridComposer.h>
#include "WAPCanvas.h"


class MainView : public gui::View
{
protected:
    WAPCanvas _canvas;
    gui::GridLayout _gl;

public:
    MainView()
        : _canvas()
        , _gl(1, 1)
    {
        gui::GridComposer gc(_gl);
        gc.appendRow(_canvas);
        setLayout(&_gl);

        _canvas.init();
    }

    void focusOnCanvas()
    {
        _canvas.setFocus(true);
    }
};