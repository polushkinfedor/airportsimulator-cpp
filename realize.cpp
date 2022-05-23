// Created by Fedor on 16.05.2022.
#include "realize.h"

TAObject::TAObject() {}
TAObject::~TAObject() {

}
TAObject::TAObject(float x, float y) {
    this->x=x;
    this->y=y;
}

TLA::TLA(float x, float y, float v, float xc, float yc):TAObject(x, y) {
    this->v = v;
    this->xc = xc;
    this->yc = yc;
    this->f = true;
    this->landing = false;
    this->landed = false;

    int b=0;
    if (this->x > this->xc) b = 0;
    else if (this->x < this->xc and this->y < this->yc) b = -1;
    else if (this->x < this->xc and this->y > this->yc) b = 1;

    this->fi=b*M_PI+atan((y-yc)/(x-xc));
    this->r = sqrt(pow((this->x - this->xc),2) + pow((this->y - this->yc),2));
}
void TLA::move(float t, int a) {
    if (a == 0) this->f = false;
    else if (a == 1) this->f = true;
}

TAircraft::TAircraft(float x, float y, float v, float xc, float yc): TLA(x,y,v,xc,yc) {}
THelicopter::THelicopter(float x, float y, float v, float xc, float yc): TLA(x,y,v,xc,yc) {}

void TAircraft::move(float t, int a) {
    if (a == 0) this->f = false;
    else if (a == 1) this->f = true;

    float omega = v/r;
    x = -(xc+r*cos(fi+omega*t))*(a-1) + a*(x+v*dt);
    y = -(yc+r*sin(fi+omega*t))*(a-1) + a*y;

    if (x > xc - v*dt and x < xc + v*dt and y < yc + v*dt and y > yc - v*dt) landed = true;
    else landed = false;
}
void THelicopter::move(float t, int a) {
    x=x-a*v*cos(fi)*dt;
    y=y-a*v*sin(fi)*dt;
    if (abs(x-xc)<v*dt and abs(y-yc)<v*dt) landed = true;
    else landed = false;
}

TAirport::TAirport(float x, float y, float l): TAObject(x, y) {
    srand(time(0)); rand();
    this->l = l;
    this->f = false;
    LA = new TLA* [K];
    for (int i(0); i<K; i++) {
        if(rand()%2 == 0) LA[i] = new TAircraft(rand()%10000+10000,rand()%10000+10000,rand()%300+600,this->x,this->y);
        else LA[i] = new THelicopter(rand()%10000+10000,rand()%10000+10000,rand()%50+300,this->x,this->y);
    }
}

void TAirport::show(char* a, int* b, float* L1, float* L2, float* L3, float* LA_land_end) {
    fstream HEREANSWER;
    HEREANSWER.open("/Users/fedor/CLionProjects/AhahAK47/cmake-build-debug/output");
    HEREANSWER << "Моделирование работы диспетчерской (ООП)" << endl;
    HEREANSWER << "Fedor Polushkin M70-107C-21" << endl;
    HEREANSWER << "В моделировании участвовали два типа летательных аппаратов: самолеты и вертолеты. Координаты" << endl;
    HEREANSWER << "аэродрома, а так же количество самолетов задано заранее, так же программе на вход подается t0 и tk" << endl;
    HEREANSWER << "Набор лететальных аппаратов создается при помощи рандомной генерации. Так в нашем конкретном случае "
               << endl;
    HEREANSWER << "система сгенерировала следующие стартовые условия: " << endl;
    for (int i(0); i < K; i++) {
        HEREANSWER << i+1 << ": " << L1[i] << " " << L2[i] << " " << L3[i] << " ";
        if (L3[i] > 450) HEREANSWER << " its Aircraft";
        else HEREANSWER << " its Helicopter";
        HEREANSWER << endl;
    }
    HEREANSWER << endl << "Координаты аэродрома("<<LA[0]->xc<<";"<<LA[0]->yc<<")"<<" константа dt равняется " << dt << endl;
    HEREANSWER << endl;
    HEREANSWER << "После выполнения работы программы все ЛА оказываются в окрестности аэропорта +- погрешность равная" << endl;
    HEREANSWER << "скорости конкретного летательного аппарата умноженная на заданнаю константу dt" << endl;
    for (int i(0); i < K; i++) {
        HEREANSWER << i+1 << ": " << LA[i]->x << " " << LA[i]->y << " " << LA[i]->v << " ";
        HEREANSWER << endl;
    }
    HEREANSWER << endl;
    HEREANSWER << "ответом к врианту 1 являются следующие данные:" << endl;
    for (int j(0); j < K; j++) {
        HEREANSWER << "TLA number " << b[j] << " landed by number " << j+1 << " and its ";
        if (a[j] == 'a') HEREANSWER << "Aircraft";
        else HEREANSWER << "Helicopter";
        HEREANSWER << endl;
    }
    HEREANSWER << endl;
    HEREANSWER << "ответом к варианту 2 являются следующие данные:" << endl;
    for (int j(0); j < K; j++) {
        HEREANSWER << "TLA number " << b[j] << " landed at " << LA_land_end[b[j]-1] << " and its ";
        if (a[j] == 'a') HEREANSWER << "Aircraft";
        else HEREANSWER << "Helicopter";
        HEREANSWER << endl;
    }
    HEREANSWER.close();
}

void TAirport::Do (float t0, float tk) {
    float LA_landing_end[K];

    // КОД МЕЖДУ ЭТИМИ КОММЕНТАРИЯМИ НЕ НЕСЕТ СМЫСЛОВОЙ НАГРУЗКИ И ИСПОЛЬЗУЕТСЯ ДЛЯ КРАСИВОГО ВЫВОДА В SHOW()
    float LA_data[K];
    float LA_data2[K];
    float LA_data3[K];
    for (int i(0); i < K; i++) {
        LA_landing_end[i] = float(0); // А ВОТ ЭТО ПОЛЕЗНО НО ВЫНОСИТЬ В ОТДЕЛЬНЫЙ КОД НЕ ИМЕЕТ СМЫСЛА
        LA_data[i] = LA[i]->x;
        LA_data2[i] = LA[i]->y;
        LA_data3[i] = LA[i]->v;
    }
    int A[K];
    char N[K];
    // КОД МЕЖДУ ЭТИМИ КОММЕНТАРИЯМИ НЕ НЕСЕТ СМЫСЛОВОЙ НАГРУЗКИ И ИСПОЛЬЗУЕТСЯ ДЛЯ КРАСИВОГО ВЫВОДА В SHOW()

    int a;
    int counter = 0;
    float X = this->x + 1.1 * this->l;
    for (float t(t0); t < tk; t += dt) {
        for (int i(0); i < K; i++) {
            if (LA[i]->landed) continue;

            if (LA[i]->v > 450) { // это самолёт
                if ((not(this->f and not LA[i]->f)) and (LA[i]->x < X) and (abs(LA[i]->y - this->y) < l / 50)) a = 1;
                else a = 0;

                if ((LA[i]->x > this->x + this->l) and LA[i]->f) {
                    LA[i]->landing = true;
                } else if (f == true) LA[i]->landing = false;
            }

            else { // рутутутуту а это вертолёт
                if (not(this->f and not(LA[i]->f))) a = 1;
                else a = 0;

                if ((pow((LA[i]->x - this->x), 2) + pow((LA[i]->y - this->y), 2) < pow((this->l / 50), 2)) and
                    (LA[i]->f)) {
                    LA[i]->landing = true;
                } else LA[i]->landing = false;
            }

            if (LA[i]->landing) {
                this->f = true;
            }

            LA[i]->move(t, a);

            if (LA[i]->landed) {
                this->f = false;
                A[counter] = i + 1;
                if (LA[i]->v > 450) N[counter] = 'a';
                else N[counter] = 'h';
                counter++;
                LA_landing_end[i] = t;
            }
        }
    }
    show(N, A, LA_data, LA_data2, LA_data3, LA_landing_end);
}