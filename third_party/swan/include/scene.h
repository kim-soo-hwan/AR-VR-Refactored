#ifndef _SCENE_H_
#define _SCENE_H_

// std
#include <vector>
#include <string>
#include <memory>
using namespace std;

// swan
#include <model.h>
#include <camera.h>
#include <axes.h>

class Scene
{
public:
    // constructor
    Scene();

    // destructor
    virtual ~Scene();

    // add model
    void addModel(const shared_ptr<Model> &model);

    // set camera
    void setCamera(const shared_ptr<Camera> &camera);

    // set axes
    void setGlobalAxes(const float scale);
    void setGlobalAxes(const shared_ptr<Axes> &axes);
    void addAxes(const shared_ptr<Axes> &axes);

    // draw
    void draw() const;

protected:
    // models
    vector<shared_ptr<Model>> models_;

    // axes
    shared_ptr<Axes> globalAxes_;
    vector<shared_ptr<Axes>> axes_;

    // camera
    shared_ptr<Camera> camera_;

    // transformation name
    string transformationMatrixName_;
};

#endif // _SCENE_H_