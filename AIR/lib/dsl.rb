# lib/dsl.rb
module AirClone
  class DSL
    attr_reader :scenes

    def initialize
      @scenes = {}
    end

    # Déclare une scène complète
    def scene(name, &block)
      builder = SceneBuilder.new(name)
      builder.instance_eval(&block)
      @scenes[name] = builder.build
    end

    class SceneBuilder
      def initialize(name)
        @name = name
        @lines = []
        @choices = []
        @background = nil
        @bgm = nil
        @next_scene = nil
      end

      def background(bg) = @background = bg
      def bgm(song) = @bgm = song

      # Ajoute une ligne de dialogue
      def say(character, text)
        @lines << { type: :dialogue, character: character, text: text }
      end

      # Narration (sans personnage)
      def narrate(text)
        @lines << { type: :narration, text: text }
      end

      # Choix du joueur
      def choice(text, goto:, flag: nil, value: true)
        @choices << { text: text, goto: goto, flag: flag, value: value }
      end

      # Scène suivante (linéaire)
      def next_scene(name) = @next_scene = name

      def build
        Scene.new(
          name: @name,
          background: @background,
          bgm: @bgm,
          lines: @lines,
          choices: @choices,
          next_scene: @next_scene
        )
      end
    end
  end
end