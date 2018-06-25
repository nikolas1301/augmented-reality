# augmented-reality
Matlab program to create a 3D object in a 2D plan.

English:
This project contains 5 codes that work independently. To run one of the codes it is necessary to have the libraries geom3D and geom2D.
To use all the codes it is necessary to do the camera calibration using the camera calibration app provided by matlab, and then you need to read this cameraParams object. It is necessary to have the webcam driver to utilize the webcam in matlab.

Portugues:
Esse projeto contem 5 códigos que funcionam independentemente. Para utilizar um dos códigos é necessario ter as bibliotecas geom3D e geom2D. Para utilizar todos os códigos é necessario ser feita a calibração da câmera atraves da aplicação de calibração de câmera do matlab e ler o objeto do tipo cameraParams. É necessario tambem ter instalado o driver para utilizar uma webcam no matlab.

Funcionamento:
O programa inicia realizando a busca dos parâmetros da câmera, identificados realizando a calibração desta, e salvado em uma variável a
matriz intrínseca da câmera, para ser utilizada na construção 3D dos objetos. Após definimos um objeto do tipo webcam e os parâmetros do 
objeto 3D, como tamanho da sua aresta, em alguns casos a rotação e a posição inicial, com isto feito inicia-se um laço de repetição para 
exibir a imagem da câmera. Dentro deste laço realizamos a retirada da distorção da imagem original e transformamos a imagem em preto e 
branco, para que as operações sejam mais rápidas de realizar, já que uma imagem preta e branca tem apenas uma dimensão, enquanto imagens 
coloridas têm três dimensões. Então encontramos o checkerboard e, com o conhecimento do tamanho dos quadrados, definimos cada ponto com 
seu valor no mundo real, e com os valores em escala de pixeis e de milímetro é possível utilizar um algoritmo de Perspective-n-Point para 
encontrar a matriz extrínseca da câmera, que representa a orientação e translação do checkerboard em relação a câmera. Com a matriz 
intrínseca e extrínseca é possível encontrar a matriz da câmera, que relaciona um ponto no mundo 3D para o espaço da imagem 2D, e assim 
podemos calcular o valor dos vértices de um cubo 3D para o espaço de pixeis. Esse cálculo é feito com os seguintes passos, multiplica-se o
ponto com valores reais de x, y e z pela matriz transposta da câmera e assim se obtêm os pontos em coordenadas homogêneas, então esses 
pontos são transformados para coordenada cartesiana e por fim estes são utilizados como vértices para plotar os quadrados que formam o cubo. No caso de imagens mais complexas foi utilizada a biblioteca geom3D, que realiza o mesh em uma curva 3D criada para poder plotar os diversos vértices e obter o objeto final.
