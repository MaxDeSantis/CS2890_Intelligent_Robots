

import soccerbot_main
import game_state
import visualizer


gameState = game_state.GameState()
visualizer = visualizer.Visualizer(gameState)
soccerBot = soccerbot_main.SoccerBot(gameState)

