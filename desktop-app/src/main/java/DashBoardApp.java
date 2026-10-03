import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.layout.StackPane;
import javafx.scene.web.WebEngine;
import javafx.scene.web.WebView;
import javafx.scene.image.Image;
import javafx.stage.Stage;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.URL;

public class DashBoardApp extends Application {

    private WebEngine webEngine;

    @Override
    public void start(Stage primaryStage) {
        Image TB_zenith = new Image(getClass().getResourceAsStream("/interface/assets/Logo_Zenith_Dashboard.png"));
        primaryStage.getIcons().add(TB_zenith);
        WebView webView = new WebView();
        this.webEngine = webView.getEngine();
        webEngine.getLoadWorker().stateProperty().addListener((observable, oldValue, newValue) -> {
            if (newValue == javafx.concurrent.Worker.State.SUCCEEDED) {
                System.out.println("HTML carregado");
                iniciarReceptorUDP();
            }
        });

        try {
            URL url = getClass().getResource("/Interface/Zenith_dash.html");
            if (url != null) {
                webEngine.load(url.toExternalForm());
            } else {
                System.out.println("ERRO: HTML não encontrado.");
                webEngine.loadContent("<h1 style='color:red;'>Erro: HTML não encontrado</h1>");
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        StackPane root = new StackPane();
        root.getChildren().add(webView);

        Scene scene = new Scene(root, 1280, 720);
        primaryStage.setScene(scene);
        primaryStage.setTitle("Zenith Dashboard");
        primaryStage.setMaximized(true); // Abre em tela cheia adaptável
        primaryStage.setFullScreen(true);

        primaryStage.show();
    }


    private void iniciarReceptorUDP() {
        Thread threadReceptor = new Thread(() -> {
            System.out.println("Receptor UDP iniciado em segundo plano na porta 9999!!");

            try (DatagramSocket socket = new DatagramSocket(9999)) {
                byte[] buffer = new byte[1024];

                while (true) {
                    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                    socket.receive(packet);

                    String mensagemRecebida = new String(packet.getData(), 0, packet.getLength()).trim();
                    //Envia o dado para o HTML
                    Platform.runLater(() -> {
                        try {
                            // Chama a função JS criada no HTML, injetando a string que veio do robô
                            System.out.println(mensagemRecebida);
                            webEngine.executeScript("atualizarMotor('elevador', '" + mensagemRecebida + "');");
                        } catch (Exception e) {
                            System.err.println("Aviso: Função JS não encontrada no HTML ou erro na WebView.");
                        }
                    });
                }
            } catch (Exception e) {
                System.err.println("Erro no receptor: " + e.getMessage());
            }
        });

        // Configura a thread para morrer automaticamente se o usuário fechar a janela do app
        threadReceptor.setDaemon(true);
        threadReceptor.start();
    }
}