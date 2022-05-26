// std
#include <string>
using namespace std;

// swan
#include <scene.h>

// constructor
Scene::Scene()
{
}

// destructor
Scene::~Scene()
{
}

// add model
void Scene::addModel(const shared_ptr<Model> &model)
{
    models_.push_back(model);
}

// set camera
void Scene::setCamera(const shared_ptr<Camera> &camera)
{
    camera_ = camera;
}

// draw
void Scene::draw() const
{
    // camera
    if (camera_)
    {
        // view            matrix: C_T_G
        // projection      matrix: NDC_T_C
        // view-projection matrix: F_T_G = NDC_T_C * C_T_G
        glm::mat4 viewProjectionMatrix = camera_->getViewProjectionMatrix();

        // for each model
        for(const auto& model : models_)
        {
            // model
            if(model) model->draw(viewProjectionMatrix);
        }

        // axes
        if(axes_) axes_->draw(viewProjectionMatrix);
    }
    else
    {
        // for each model
        for(const auto& model : models_)
        {
            // model
            if(model) model->draw();
        }
    }
}

void Scene::setAxes(const shared_ptr<Axes> &axes)
{
    axes_ = axes;
}