#include <scene.h>

// constructor
Scene::Scene()
{
    // default camera
    camera_ = make_shared<Camera>();
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
bool Scene::draw() const
{
    // camera
    if (!camera_) return false;

    // view            matrix: C_T_G
    // projection      matrix: F_T_C
    // view-projection matrix: F_T_G = F_T_C * C_T_G
    glm::mat4 viewProjectionMatrix = camera_->getViewProjectionMatrix();

    // for each model
    for(const auto& model : models)
    {
        // model
        if(model) model->draw(viewProjectionMatrix);
    }
}