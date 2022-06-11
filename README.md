# Projeto Embarcados

Desenvolvendo um controle remoto.

### Integrantes

- Fabricio Neri Lima
- Jean Silas Ferreira Sanandrez

### Ideia

A ideia consiste em criar um controle para o jogo CS (Counter Strike). Por meio de sistemas embarcados, pretendemos criar um controle mais imersivo, em que o jogador não precisa do teclado para jogar, podendo atirar, agachar, trocar de arma, e se locomover através do novo controle. 

### Nome

IMECON (Imersive Control)

### Usuários 

Jogadores de Counter Strike que procuram uma nova mecânica de jogabilidade mais imersiva para o seu jogo favorito, fungindo do teclado e do mouse.

### Software/Jogo 

O software a ser controlado é o jogo Counter Strike para PC.

### Jornada do usuários

Jornada do Pedro:

Pedro é um jogador destro, então para jogar uma partida, utilizará o controle IMECON. Com seu lado esquerdo, ele consegue usar um equipamente para recarregar a arma e trocá-la, enquanto que com o lado direito ele consegue atirar, transladar e rotacionar. Assim, ele consegue cumprir as funcionalidades principais do jogo com o controle IMECON.


Para agachar, o Pedro utiliza seu braço esquerdo, movendo o para o lado Já para jogar, utiliza o gatilho do controle, e para se mover, move o analógico. O movimento da visão do jogador é feito por Pedro ao mover rapidamente o controle para um dos lados.

Jornada do João:

João é um jogador canhoto, então para jogar uma partida, utilizará o controle IMECON. Com seu lado direito, ele consegue usar um equipamente agachar, enquanto que com o lado esquerdo ele consegue atirar, transladar e rotacionar. Assim, ele consegue cumprir as funcionalidades principais do jogo com o controle IMECON.

Para agachar, o João utiliza seu braço direito, movendo o para o lado, enquanto que para trocar de arma, move o mesmo braço para o outro lado. Já para jogar, utiliza o gatilho do controle, e para se mover, move o analógico. O movimento da visão do jogador é feito por João ao mover rapidamente o controle para um dos lados.

### Comandos/ Feedbacks 

**Comandos e operações** 

Atirar, agachar, pular, se locomover (transladar e rotacionar) e abrir o menu.

**Feedbacks**

Vibração e led piscando de acordo com o dano recebido.

## In/OUT 


Para cada Comando/ Feedback do seu controle, associe qual sensores/ atuadores pretende utilizar? Faca em formato de lista, exemplo:

- Atira/Apunhala com faca: Aperta o trigger/botão.
    Racional: como atirar é uma ação binária e requere rapidez e destreza, utilizar um trigger parece bem coerente apropiado. Esse triger estaria na mão direita, se o usuário for destro e na mão direita caso o usuário seja canhoto. 

- agachar a arma: utiliza o IMU.
  Racional: como trocar de arma é uma ação que envolve um comando de somente um input, então utilizar os parâmetros de movimento para identificar um comando para recarregar é coerente com a experiência imersiva a qual buscamos entregar ao usuário. Nesse sentido,  o usuário faria um gesto com a mão esquerda ( se destro e direita se canhoto), que se assemelha o recarregar de uma arma.


- Locomover(translado): utiliza joypad analógico.
    Racional: Como se locomover é um comando que envolve uma entrada de sinal contínuo, logo utilizar o analógico joystick faz sentido para que o usuário possa transladar o seu player pelo cenário do jogo. O controle analógico estaria na mão direita, caso o usuário seja destro e na mão esquerda caso ele seja canhoto.

- Locomover(rotacional): utiliza o IMU.
    Racional: Como se locomover roatacionalmente em referência ao seu próprio eixo é algo que exige um sinal contínuo, então fariamos o uso de IMU associado ao controle para poder mudar a visão de mira do jogador ( isso seria algo parecido com a mecânica de jogabilidade dos controle do wii)

- Abrir o Menu: utiliza um botão
    Racional: Como abrir ou fechar o menu é uma ação binária, logo utilizar a mecânica de aplicar pressão em um botão faz sentido.
    
- Pular: utiliza um botão
    Racional: Pular é uma ação binária, logo, apenas apertar um botão é algo que já iria satisfazer.

### Video
[![Game Play](https://user-images.githubusercontent.com/39420630/173192694-505a1d99-1187-4929-9e7b-8719f6dddf81.png)](https://www.youtube.com/watch?v=3tfhDoeAuwk&ab_channel=Fabr%C3%ADcioNeri)

### Design (2 pts)
![WhatsApp Image 2022-03-25 at 18 24 21](https://user-images.githubusercontent.com/39420630/160203064-91dcde8b-8e27-4d75-9cf4-bc7833bdbbdd.jpeg)
A imagem apresenta os items principais do projeto, porém não mostra todos os componentes utilizados, pois alguns estarão internamente.

### Design Final

![controle_do_wii](https://user-images.githubusercontent.com/39682690/167319632-21b13314-5228-4b51-a23d-0ccc7e90e634.png)

