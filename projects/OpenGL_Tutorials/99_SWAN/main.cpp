int main()
{
    float vertex_input[] = {};

    Window window;

    Model triangle;
    triangle.setVertexInput(vertex_input);
    triangle.setAttribute(0, 3, );
    triangle.setAttribute(1, 3, );
    triangle.setAttribute(2, 4, );
    triangle.setTexture();
    triangle.setUniform();

    Shader shader("test.vs", "test.fs");
    Camera camera(window, shader);

    while(!window.shouldclose()) 
    {
        window.wipeout();

        triangle.draw(shader);
        rectangle.draw()

        window.display();
        window.keyboard();
    }

    return 0;
}