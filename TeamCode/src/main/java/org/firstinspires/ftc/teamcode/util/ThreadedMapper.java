package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.CompletableFuture;

public interface ThreadedMapper {
    default GamepadMapSingle mapGamepadSingle(Gamepad gamepad) {
        return new GamepadMapSingle(detectMap(gamepad));
    }

    default GamepadMapDual mapGamepadDual(Gamepad gamepad1, Gamepad gamepad2) {
        CompletableFuture<GamepadMap> mapPad1 = CompletableFuture.supplyAsync(() -> detectMap(gamepad1));
        CompletableFuture<GamepadMap> mapPad2 = CompletableFuture.supplyAsync(() -> detectMap(gamepad2));
        CompletableFuture<GamepadMapDual> dualMap = mapPad1.thenCombine(mapPad2, GamepadMapDual::new);
        return dualMap.join();
    }

    default GamepadMap detectMap(Gamepad gamepad) {
        return gamepad.left_trigger <= 0.1 ? GamepadMap.BACKUP : GamepadMap.STANDARD;
    }

    class GamepadMapSingle {
        public GamepadMap map;
        public GamepadMapSingle(GamepadMap map) {
            this.map = map;
        }
    }

    class GamepadMapDual {
        public GamepadMap map1;
        public GamepadMap map2;
        public GamepadMapDual(GamepadMap map1, GamepadMap map2) {
            this.map1 = map1;
            this.map2 = map2;
        }
    }
}
