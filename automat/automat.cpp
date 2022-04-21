#include<iostream>
#include<GL/glew.h>
#include<GLFW/glfw3.h>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/mat4x4.hpp>
#include<chrono>
#include<cmath>
#include<vector>

#include "SOIL.h"

#pragma comment(lib, "glfw3.lib")
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "opengl32.lib")

using namespace std;
using namespace glm;

GLFWwindow* g_window;

const char* filename = "C:\\Users\\Михаил\\Desktop\\Графика и вычислительная геометрия\\горшок.png";

GLuint g_shaderProgramPoint, g_shaderProgramLine, g_shaderProgramFigure;
float scale_coef = 5.f;

GLint mapLocation1;
GLuint texID;

GLfloat g_uColors;
GLfloat colors[258];
GLint g_MV;
GLint g_uMVP;
GLint g_uMV;
mat4 ModelMatrix = mat4(1.0f);
GLfloat angle = 0.0f;


chrono::system_clock::time_point old_time = chrono::system_clock::now();
chrono::system_clock::time_point cur_time = chrono::system_clock::now();

enum states {DRAW_CIRCLE, DRAW_FIGURE_ROTATE};

states state;

class Model
{
public:
    GLuint vbo;
    GLuint ibo;
    GLuint vao;
    GLsizei indexCount;
};

/**
 * Threshold for zero.
 */
#define EPSILON 1.0e-5
 /**
  * Test if real value is zero.
  */
#define IS_ZERO(v) (abs(v) < EPSILON)
  /**
   * Signum function.
   */
#define SIGN(v) (int)(((v) > EPSILON) - ((v) < -EPSILON))
   /**
    * Amount of lines representing each Bezier segment.
    */
#define RESOLUTION 32
    /**
     * Paramenet affecting curvature, should be in [2; +inf).
     */
#define C 2.0

class Point
{
public:
    double x, y;
    Point(double _x, double _y) {
        x = _x;
        y = _y;
    }

    Point() { x = y = 0.0; };

    Point operator +(const Point& p) const { return Point(x + p.x, y + p.y); };

    Point operator -(const Point& p) const { return Point(x - p.x, y - p.y); };

    Point operator *(double v) const { return Point(x * v, y * v); };

    void normalize()
    {
        double l = sqrt(x * x + y * y);
        if (IS_ZERO(l))
            x = y = 0.0;
        else
        {
            x /= l;
            y /= l;
        }
    };

    static Point absMin(const Point& p1, const Point& p2)
    {
        return Point(abs(p1.x) < abs(p2.x) ? p1.x : p2.x, abs(p1.y) < abs(p2.y) ? p1.y : p2.y);
    };
};

vector<Point> points;
vector<Point> bezie_points;

class Segment
{
public:
    /**
     * Bezier control points.
     */
    Point points[4];

    /**
     * Calculate the intermediate curve points.
     *
     * @param t - parameter of the curve, should be in [0; 1].
     * @return intermediate Bezier curve point that corresponds the given parameter.
     */
    Point calc(double t) const
    {
        double t2 = t * t;
        double t3 = t2 * t;
        double nt = 1.0 - t;
        double nt2 = nt * nt;
        double nt3 = nt2 * nt;
        return Point(nt3 * points[0].x + 3.0 * t * nt2 * points[1].x + 3.0 * t2 * nt * points[2].x + t3 * points[3].x,
            nt3 * points[0].y + 3.0 * t * nt2 * points[1].y + 3.0 * t2 * nt * points[2].y + t3 * points[3].y);
    };
};

bool tbezierSO1(const vector<Point>& values, vector<Segment>& curve)
{
    int n = values.size() - 1;

    if (n < 2)
        return false;

    curve.resize(n);

    Point cur, next, tgL, tgR, deltaL, deltaC, deltaR;
    double l1, l2;

    next = values[1] - values[0];
    next.normalize();

    for (int i = 0; i < n; ++i)
    {
        tgL = tgR;
        cur = next;

        deltaC = values[i + 1] - values[i];

        if (i > 0)
            deltaL = Point::absMin(deltaC, values[i] - values[i - 1]);
        else
            deltaL = deltaC;

        if (i < n - 1)
        {
            next = values[i + 2] - values[i + 1];
            next.normalize();
            if (IS_ZERO(cur.x) || IS_ZERO(cur.y))
                tgR = cur;
            else if (IS_ZERO(next.x) || IS_ZERO(next.y))
                tgR = next;
            else
                tgR = cur + next;
            tgR.normalize();
            deltaR = Point::absMin(deltaC, values[i + 2] - values[i + 1]);
        }
        else
        {
            tgR = Point();
            deltaR = deltaC;
        }

        l1 = IS_ZERO(tgL.x) ? 0.0 : deltaL.x / (C * tgL.x);
        l2 = IS_ZERO(tgR.x) ? 0.0 : deltaR.x / (C * tgR.x);

        if (abs(l1 * tgL.y) > abs(deltaL.y))
            l1 = IS_ZERO(tgL.y) ? 0.0 : deltaL.y / tgL.y;
        if (abs(l2 * tgR.y) > abs(deltaR.y))
            l2 = IS_ZERO(tgR.y) ? 0.0 : deltaR.y / tgR.y;

        curve[i].points[0] = values[i];
        curve[i].points[1] = curve[i].points[0] + tgL * l1;
        curve[i].points[3] = values[i + 1];
        curve[i].points[2] = curve[i].points[3] - tgR * l2;
    }

    return true;
}

bool tbezierSO0(const vector<Point>& values, vector<Segment>& curve)
{
    int n = values.size() - 1;

    if (n < 2)
        return false;

    curve.resize(n);

    Point cur, next, tgL, tgR, deltaC;
    double l1, l2, tmp, x;
    bool zL, zR;

    next = values[1] - values[0];
    next.normalize();

    for (int i = 0; i < n; ++i)
    {
        tgL = tgR;
        cur = next;

        deltaC = values[i + 1] - values[i];

        if (i < n - 1)
        {
            next = values[i + 2] - values[i + 1];
            next.normalize();
            if (IS_ZERO(cur.x) || IS_ZERO(cur.y))
                tgR = cur;
            else if (IS_ZERO(next.x) || IS_ZERO(next.y))
                tgR = next;
            else
                tgR = cur + next;
            tgR.normalize();
        }
        else
        {
            tgR = Point();
        }

        // There is actually a little mistake in the white paper (http://sv-journal.org/2017-1/04.php?lang=en):
        // algorithm described after figure 14 implicitly assumes that tangent vectors point inside the
        // A_i and B_i areas (see fig. 14). However in practice they can point outside as well. If so, tangents’
        // coordinates should be clamped to the border of A_i or B_i respectively to keep control points inside
        // the described area and thereby to avoid false extremes and loops on the curve.
        // The clamping is implemented by the next 4 if-statements.
        if (SIGN(tgL.x) != SIGN(deltaC.x))
            tgL.x = 0.0;
        if (SIGN(tgL.y) != SIGN(deltaC.y))
            tgL.y = 0.0;
        if (SIGN(tgR.x) != SIGN(deltaC.x))
            tgR.x = 0.0;
        if (SIGN(tgR.y) != SIGN(deltaC.y))
            tgR.y = 0.0;

        zL = IS_ZERO(tgL.x);
        zR = IS_ZERO(tgR.x);

        l1 = zL ? 0.0 : deltaC.x / (C * tgL.x);
        l2 = zR ? 0.0 : deltaC.x / (C * tgR.x);

        if (abs(l1 * tgL.y) > abs(deltaC.y))
            l1 = IS_ZERO(tgL.y) ? 0.0 : deltaC.y / tgL.y;
        if (abs(l2 * tgR.y) > abs(deltaC.y))
            l2 = IS_ZERO(tgR.y) ? 0.0 : deltaC.y / tgR.y;

        if (!zL && !zR)
        {
            tmp = tgL.y / tgL.x - tgR.y / tgR.x;
            if (!IS_ZERO(tmp))
            {
                x = (values[i + 1].y - tgR.y / tgR.x * values[i + 1].x - values[i].y + tgL.y / tgL.x * values[i].x) / tmp;
                if (x > values[i].x && x < values[i + 1].x)
                {
                    if (abs(l1) > abs(l2))
                        l1 = 0.0;
                    else
                        l2 = 0.0;
                }
            }
        }

        curve[i].points[0] = values[i];
        curve[i].points[1] = curve[i].points[0] + tgL * l1;
        curve[i].points[3] = values[i + 1];
        curve[i].points[2] = curve[i].points[3] - tgR * l2;
    }

    return true;
}


void bez(vector<Point> p)
{
    vector<Segment> curve;
    bezie_points.clear();

    tbezierSO0(points, curve);

    for (auto s : curve)
    {
        for (int i = 0; i < RESOLUTION; ++i)
        {
            bezie_points.push_back(s.calc((double)i / (double)RESOLUTION));
        }
    }
}


Model g_model_line, g_model_point, g_figure_model;

GLuint createShader(const GLchar* code, GLenum type)
{
    GLuint result = glCreateShader(type);

    glShaderSource(result, 1, &code, NULL);
    glCompileShader(result);

    GLint compiled;
    glGetShaderiv(result, GL_COMPILE_STATUS, &compiled);

    if (!compiled)
    {
        GLint infoLen = 0;
        glGetShaderiv(result, GL_INFO_LOG_LENGTH, &infoLen);
        if (infoLen > 0)
        {
            char* infoLog = (char*)alloca(infoLen);
            glGetShaderInfoLog(result, infoLen, NULL, infoLog);
            cout << "Shader compilation error" << endl << infoLog << endl;
        }
        glDeleteShader(result);
        return 0;
    }

    return result;
}

GLuint createProgram(GLuint vsh, GLuint fsh)
{
    GLuint result = glCreateProgram();

    glAttachShader(result, vsh);
    glAttachShader(result, fsh);

    glLinkProgram(result);

    GLint linked;
    glGetProgramiv(result, GL_LINK_STATUS, &linked);

    if (!linked)
    {
        GLint infoLen = 0;
        glGetProgramiv(result, GL_INFO_LOG_LENGTH, &infoLen);
        if (infoLen > 0)
        {
            char* infoLog = (char*)alloca(infoLen);
            glGetProgramInfoLog(result, infoLen, NULL, infoLog);
            cout << "Shader program linking error" << endl << infoLog << endl;
        }
        glDeleteProgram(result);
        return 0;
    }

    return result;
}

bool createShaderProgram()
{
    g_shaderProgramPoint = 0;
    g_shaderProgramLine = 0;

    g_shaderProgramPoint = 0;
    g_shaderProgramLine = 0;

    const GLchar vshP[] =
        "#version 330\n"
        ""
        "layout(location = 0) in vec2 a_position;"
        "uniform mat4 u_MV;"
        ""
        "void main()"
        "{"
        "    gl_Position = u_MV * vec4(a_position, 0.0, 1.0);"
        "}"
        ;

    const GLchar fshP[] =
        "#version 330\n"
        ""
        "layout(location = 0) out vec4 o_color;"
        ""
        "void main()"
        "{"
        "   o_color = vec4(1.0, 0.0, 0.0, 1.0);"
        "}"
        ;

    const GLchar vshL[] =
        "#version 330\n"
        ""
        "layout(location = 0) in vec2 a_position;"
        ""
        "void main()"
        "{"
        "    gl_Position = vec4(a_position, 0.0, 1.0);"
        "}"
        ;

    const GLchar fshL[] =
        "#version 330\n"
        ""
        "layout(location = 0) out vec4 o_color;"
        ""
        "void main()"
        "{"
        "   o_color = vec4(0.0, 1.0, 0.0, 1.0);"
        "}"
        ;

    const GLchar vshF[] =
        "#version 330\n"
        ""
        "layout(location = 0) in vec4 a_position;"
        "uniform mat4 u_mvp;"
        "uniform mat4 u_mv;"
        "out vec4 pos;"
        ""
        "void main()"
        "{"
        "   pos = a_position;"
        "   gl_Position = u_mvp * pos;"
        "}"
        ;

    const GLchar fshF[] =
        "#version 330\n"
        ""
        "uniform sampler2D u_map1;"
        "layout(location = 0) out vec4 o_color;"
        "in vec4 pos;"
        ""
        "void main()"
        "{"
        "   float gamma=2.2;"
        ""
        "   vec4 c1 = texture(u_map1, vec2(pos[0], pos[1]));"
        "   c1.rgb = pow(c1.rgb, vec3(2.2));"
        "   o_color = vec4(c1.rgb, 1.0);"
        "}"
        ;

    GLuint vertexShader, fragmentShader;
    if (state == DRAW_CIRCLE)
    {
        vertexShader = createShader(vshP, GL_VERTEX_SHADER);
        fragmentShader = createShader(fshP, GL_FRAGMENT_SHADER);

        g_shaderProgramPoint = createProgram(vertexShader, fragmentShader);

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);

        vertexShader = createShader(vshL, GL_VERTEX_SHADER);
        fragmentShader = createShader(fshL, GL_FRAGMENT_SHADER);

        g_shaderProgramLine = createProgram(vertexShader, fragmentShader);

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        return g_shaderProgramPoint != 0;
    }

    vertexShader = createShader(vshF, GL_VERTEX_SHADER);
    fragmentShader = createShader(fshF, GL_FRAGMENT_SHADER);

    g_shaderProgramFigure = createProgram(vertexShader, fragmentShader);

    g_uMVP = glGetUniformLocation(g_shaderProgramFigure, "u_mvp");
    g_uMV = glGetUniformLocation(g_shaderProgramFigure, "u_mv");
    mapLocation1 = glGetUniformLocation(g_shaderProgramFigure, "u_map1");
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return g_shaderProgramPoint != 0;
}

void init_tex(GLuint tex, const char* filename)
{
    GLsizei texW, texH;
    unsigned char* image = SOIL_load_image(filename, &texW, &texH, 0, SOIL_LOAD_RGBA);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texW, texH, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
    SOIL_free_image_data(image);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
}

bool createModelPoint()
{
    const GLfloat vertices[] = {
         -0.5f, -0.5,
         0.5f, -0.5f,
         0.0f, 0.5f
    };

    const GLuint indices[] = {
            0, 1, 2
    };

    glGenVertexArrays(1, &g_model_point.vao);
    glBindVertexArray(g_model_point.vao);

    glGenBuffers(1, &g_model_point.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, g_model_point.vbo);
    glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(GLfloat), vertices, GL_STATIC_DRAW);

    glGenBuffers(1, &g_model_point.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_model_point.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 3 * sizeof(GLuint), indices, GL_STATIC_DRAW);
    g_model_point.indexCount = 3;

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), (const GLvoid*)0);

    return g_model_point.vbo != 0 && g_model_point.ibo != 0 && g_model_point.vao != 0;
}

bool createModelLine()
{
    GLfloat* vertex_arr = new GLfloat[bezie_points.size() * 2];
    GLint* index_arr = new GLint[bezie_points.size()];

    int i = 0;

    for (Point p : bezie_points)
    {
        vertex_arr[i] = p.x;
        vertex_arr[i + 1] = p.y;
        index_arr[i / 2] = i / 2;
        i += 2;
    }

    glGenVertexArrays(1, &g_model_line.vao);
    glBindVertexArray(g_model_line.vao);

    glGenBuffers(1, &g_model_line.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, g_model_line.vbo);
    glBufferData(GL_ARRAY_BUFFER, bezie_points.size() * 2 * sizeof(GLfloat), vertex_arr, GL_STATIC_DRAW);

    glGenBuffers(1, &g_model_line.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_model_line.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, bezie_points.size() * sizeof(GLint), index_arr, GL_STATIC_DRAW);

    g_model_line.indexCount = bezie_points.size();

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), (const GLvoid*)0);

    delete[] vertex_arr;
    delete[] index_arr;

    return g_model_line.vbo != 0 && g_model_line.ibo != 0 && g_model_line.vao != 0;
}

bool createModelFigure()
{
    int k = bezie_points.size();
    //cout << k << endl;
    GLint cout_base = 360; // кол во базовых линий
    GLint vertex_size = k * 4 * cout_base;
    GLint index_size = (cout_base - 1) * 6 * (k - 1);
    GLfloat* vertex_arr = new GLfloat[vertex_size];
    GLint* index_arr = new GLint[index_size + 6 * (k - 1)];

    int i = 0;
    GLfloat base_angle = (360 / cout_base); // угол поворота базовых линий

    for (int j = 0; j < cout_base; j++) {
        for (Point p : bezie_points)
        {
            vec4 v(p.x, p.y, 0.0, 1.0);
            GLfloat a = base_angle * j;
            v = rotate(mat4(1.0f), radians(a), vec3(0, 1, 0)) * v;

            vertex_arr[i] = v[0];
            vertex_arr[i + 1] = v[1];
            vertex_arr[i + 2] = v[2];
            vertex_arr[i + 3] = v[3];
            //cout <<"x_y_z "<< vertex_arr[i] << " " << vertex_arr[i + 1] << " " << vertex_arr[i + 2] << endl;
            i += 4;
        }

    }


    i = 0;
    int q = 1;
    for (int j = 0; j < index_size; j+=6)
    {
        index_arr[j] = i;
        index_arr[j + 1] = i + 1;
        index_arr[j + 2] = i + k + 1;

        //cout << index_arr[j] << " "<< index_arr[j + 1] << " " << index_arr[j + 2]<< " ";

        index_arr[j + 3] = i + k + 1;
        index_arr[j + 4] = i + k;
        index_arr[j + 5] = i;

        //cout << index_arr[j + 3] << " " << index_arr[j + 4] << " " << index_arr[j + 5] << endl;
        i++;
        /*что бы не было протяжек плоскостей друг от друга*/
        if (i == k *q - 1) {
            i++;
            q++;
        }
    }

    i = 0;
     /*заделываю дырку в текстуре*/
    for (int j = index_size; j < index_size + 6 * (k - 1); j+=6)
    {
        index_arr[j] = i;
        index_arr[j + 1] = i + 1;
        index_arr[j + 2] = (cout_base - 1)*k + i + 1;

        index_arr[j + 3] = (cout_base - 1) * k + i + 1;
        index_arr[j + 4] = (cout_base - 1) * k + i;
        index_arr[j + 5] = i; 
        i++;
    }

    glGenVertexArrays(1, &g_figure_model.vao);
    glBindVertexArray(g_figure_model.vao);

    glGenBuffers(1, &g_figure_model.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, g_figure_model.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertex_size * sizeof(GLfloat), vertex_arr, GL_STATIC_DRAW);

    glGenBuffers(1, &g_figure_model.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_figure_model.ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (index_size + 6 * (k - 1)) * sizeof(GLint), index_arr, GL_STATIC_DRAW);

    g_figure_model.indexCount = (index_size + 6 * (k - 1));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), (const GLvoid*)0);

    delete[] vertex_arr;
    delete[] index_arr;

    return g_figure_model.vbo != 0 && g_figure_model.ibo != 0 && g_figure_model.vao != 0;

}

bool init()
{
    // Set initial color of color buffer to white.
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    if (state == DRAW_FIGURE_ROTATE) {
        glEnable(GL_DEPTH_TEST); // изменение состояний в машине состояний 
        glGenTextures(1, &texID);
        init_tex(texID, filename);
    }
    // подключает тест глубины

    return createShaderProgram() && createModelLine();
}

void reshape(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void draw()
{
    // Clear color buffer.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    if (state == DRAW_CIRCLE) {
        /*отрисовка в 2D формате точек и линий безье*/
        glUseProgram(g_shaderProgramPoint);
        glBindVertexArray(g_model_point.vao);

        for (Point point : points) { 
            /*фокус с точками в том что я просто изначально в модели прокидываю точку в определеном месте
            а потом матрицами переноса в которых заложена координата клика переношу их*/
            mat4 view = translate(mat4(1.0), vec3(point.x, point.y, 0.0f));
            view = scale(view, vec3(0.05f));
            glUniformMatrix4fv(g_MV, 1, GL_FALSE, &view[0][0]);
            glDrawElements(GL_TRIANGLES, g_model_point.indexCount, GL_UNSIGNED_INT, NULL);
        }


        glUseProgram(g_shaderProgramLine);
        glBindVertexArray(g_model_line.vao);
        /*с линиями же поступаю по людски*/
        glDrawElements(GL_LINE_STRIP, g_model_line.indexCount, GL_UNSIGNED_INT, NULL);
    }
    else
    {
        /*отрисовка фигуры в 3D*/
        glUseProgram(g_shaderProgramFigure);
        glBindVertexArray(g_figure_model.vao);

        mat4 Projection = perspective(
            radians(45.0f), // Угол обзора
            1.0f,   // Соотншение сторон окна w/h
            0.01f,             // Ближняя плоскость отсечения
            1000.0f         // Дальняя плоскость отсечения
        );

        mat4 View = lookAt(
            vec3(5, 8, 5), // Координаты камеры
            vec3(0, 0, 0), // Направление камеры в начало координат
            vec3(0, 1, 0)  // Координаты наблюдателя
        );
        /*масштабируем фигуру*/
        View = scale(View, vec3(3.5f));


        /* Матрица MV*/
        cur_time = chrono::system_clock::now();
        angle = fmod(3.14, pow(1 + chrono::duration_cast<chrono::microseconds>(cur_time - old_time).count(), 3.0)) / 300;
        /* тут заложена функция расчета поворота
        суть в том чем дольше рабтал предыуший шейдер тем меньше будет угол поворота во избежания рывков при анимации*/
        old_time = cur_time;

        ModelMatrix = rotate(ModelMatrix, radians(angle), vec3(0, 1, 0));
        mat4 MV = View * ModelMatrix;

        /* Матрица MVP*/
        mat4 MVP = Projection * MV;
        float time = chrono::duration_cast<chrono::microseconds>(cur_time - old_time).count();

        mat3 M3_3 = transpose(inverse(mat3(MV)));

        /*отправляем её в шейдер*/
        glUniformMatrix4fv(g_uMV, 1, GL_FALSE, &MV[0][0]);
        glUniformMatrix4fv(g_uMVP, 1, GL_FALSE, &MVP[0][0]);

        // текстуру(что бы было)
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texID);
        glUniform1i(mapLocation1, 0);

        /*вызываем перерисовку*/
        glDrawElements(GL_TRIANGLES, g_figure_model.indexCount, GL_UNSIGNED_INT, NULL);
    }
}

void cleanup()
{
    if (g_shaderProgramLine != 0)
        glDeleteProgram(g_shaderProgramLine);

    if (g_shaderProgramPoint != 0)
        glDeleteProgram(g_shaderProgramPoint);


    if (g_model_line.vbo != 0)
        glDeleteBuffers(1, &g_model_line.vbo);
    if (g_model_line.ibo != 0)
        glDeleteBuffers(1, &g_model_line.ibo);
    if (g_model_line.vao != 0)
        glDeleteVertexArrays(1, &g_model_line.vao);


    if (g_model_point.vbo != 0)
        glDeleteBuffers(1, &g_model_point.vbo);
    if (g_model_point.ibo != 0)
        glDeleteBuffers(1, &g_model_point.ibo);
    if (g_model_point.vao != 0)
        glDeleteVertexArrays(1, &g_model_point.vao);

    glDeleteTextures(1, &texID);
}

bool initOpenGL()
{
    // Initialize GLFW functions.
    if (!glfwInit())
    {
        cout << "Failed to initialize GLFW" << endl;
        return false;
    }

    // Request OpenGL 3.3 without obsoleted functions.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window.
    g_window = glfwCreateWindow(800, 600, "OpenGL Test", NULL, NULL);
    if (g_window == NULL)
    {
        cout << "Failed to open GLFW window" << endl;
        glfwTerminate();
        return false;
    }

    // Initialize OpenGL context with.
    glfwMakeContextCurrent(g_window);

    // Set internal GLEW variable to activate OpenGL core profile.
    glewExperimental = true;

    // Initialize GLEW functions.
    if (glewInit() != GLEW_OK)
    {
        cout << "Failed to initialize GLEW" << endl;
        return false;
    }

    // Ensure we can capture the escape key being pressed.
    glfwSetInputMode(g_window, GLFW_STICKY_KEYS, GL_TRUE);

    // Set callback for framebuffer resizing event.
    glfwSetFramebufferSizeCallback(g_window, reshape);

    return true;
}

void tearDownOpenGL()
{
    // Terminate GLFW.
    glfwTerminate();
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
/*считывание координат точек*/
{
    if (action == GLFW_PRESS && state == DRAW_CIRCLE)
    {
        double x, y;
        glfwGetCursorPos(window, &x, &y);

        int w, h;
        glfwGetWindowSize(window, &w, &h);
        
        points.push_back(Point(x / w * 2.0 - 1.0, 1.0 - y/h *2.0));
        cout << x / w * 2.0 - 1.0 << " " << 1.0 - y / h * 2.0 << endl;

        if (points.size() == 2)
        {
            bezie_points = points;
        }

        if (points.size() > 2)
        {
            bez(points);
        }

        createModelLine();
        createModelPoint();
    }
}

int main()
{
    // Initialize OpenGL
    if (!initOpenGL())
        return -1;

    // Initialize graphical resources.
    bool isOk = init();
    glfwSetMouseButtonCallback(g_window, mouse_button_callback);
    //glfwSetKeyCallback(g_window, enter_press);

    if (isOk)
    {
        // Main loop until window closed or escape pressed.
        while (glfwGetKey(g_window, GLFW_KEY_ESCAPE) != GLFW_PRESS && glfwWindowShouldClose(g_window) == 0)
        {
            if (glfwGetKey(g_window, GLFW_KEY_SPACE) == GLFW_PRESS) {
                state = DRAW_FIGURE_ROTATE;
                createModelFigure();
                isOk = init();
            }

            // Draw scene.
            draw();

            // Swap buffers.
            glfwSwapBuffers(g_window);
            // Poll window events.
            glfwPollEvents();
        }
    }

    // Cleanup graphical resources.
    cleanup();

    // Tear down OpenGL.
    tearDownOpenGL();

    return isOk ? 0 : -1;
}
