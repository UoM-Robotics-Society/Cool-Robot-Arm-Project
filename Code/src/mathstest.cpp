#include "la.h"
using namespace LA;

int main(int argc, char *argv[]) {
    double array[] = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f };
    vecd<5> v0 = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f };
    vecd<5> v1 = vecd<5>({ 0.0f, 1.0f, 2.0f, 3.0f, 4.0f });
    v1[0] = 2.0f;
    v1[1] = 4.0f;
    matd<5, 1> m = matd<5, 1>();
    m[0] = v0;
    m[1] = v1;
    matd<1, 5> v = matd<1, 5>();
    v[0][0] = 10.0f;
    v[0][1] = 20.0f;

    vecd<5> res = m * v * v0;
    print(v0);
    print(v1);
    print(m);
    print(v);
    print(res);

    std::cout << sizeof(m[0]) << std::endl;
    std::cout << sizeof(v0) << std::endl;
    return 0;
}