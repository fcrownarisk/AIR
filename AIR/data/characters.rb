# data/characters.rb
module AirClone
  class Character
    REGISTRY = {
      yukito:  { name: "Kunisaki Yukito", color: :light_cyan,   voice: "绿川光" },
      misuzu:  { name: "Kamio Misuzu",    color: :light_yellow, voice: "川上とも子" },
      kano:    { name: "Kirisima Kano",   color: :light_green,  voice: "岡本麻見" },
      minagi:  { name: "Tohno Minagi",    color: :light_magenta,voice: "柚木涼香" },
      ryuuya:  { name: "Ryuuya",          color: :light_blue,   voice: "神奈延年" },
      kanna:   { name: "Kanna",           color: :light_red,    voice: "川澄綾子" },
      narrator:{ name: "",                color: :white,        voice: nil }
    }.freeze

    def self.name_of(key)
      REGISTRY.dig(key.to_sym, :name) || key.to_s
    end

    def self.color_of(key)
      REGISTRY.dig(key.to_sym, :color) || :white
    end

    def self.voice_of(key)
      REGISTRY.dig(key.to_sym, :voice)
    end
  end
end