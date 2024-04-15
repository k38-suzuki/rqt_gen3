/**
    @author Kenta Suzuki
*/

#ifndef rqt_gen3_lite__my_widget_H
#define rqt_gen3_lite__my_widget_H

#include <QWidget>

namespace rqt_gen3_lite {

class MyWidget : public QWidget
{
    Q_OBJECT
public:
    MyWidget(QWidget* parent = nullptr);

    virtual ~MyWidget();

private:
    class Impl;
    Impl* impl;
};

}

#endif // rqt_gen3_lite__my_widget_H
