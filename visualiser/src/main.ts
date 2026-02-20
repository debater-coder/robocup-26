import { Application, Assets, Container, Point, Sprite, Ticker } from "pixi.js";
import GUI from "lil-gui";

const WIDTH = 2430;
const HEIGHT = 1820;

const gui = new GUI();

const config = {
  connectionUrl: "",
  zoom: 0.5,
};

gui.add(config, "connectionUrl");

(async () => {
  // Create a new application
  const app = new Application();

  // Initialize the application
  await app.init({ background: "#37700C", resizeTo: window });

  gui.add(config, "zoom", 0, 1).onChange((zoom: number) => {
    app.stage.scale.set(zoom);
  });
  app.stage.scale.set(config.zoom);
  document.getElementById("pixi-container")!.appendChild(app.canvas);

  const fieldContainer = new Container();
  app.stage.addChild(fieldContainer);
  fieldContainer.position.set(WIDTH / 2, HEIGHT / 2); // Set field origin to centre

  const field = new Sprite(await Assets.load("/assets/field.png"));
  field.anchor.set(0.5);
  fieldContainer.addChild(field);

  const robotTexture = await Assets.load("/assets/bot.png");
  const robotPositions = [new Point(0, 0)];

  robotPositions.map((robot) => {
    const robotSprite = new Sprite(robotTexture);
    robotSprite.anchor.set(0.5);

    robotSprite.position.copyFrom(robot);
    fieldContainer.addChild(robotSprite);
  });

  app.ticker.add((time) => {});
})();
