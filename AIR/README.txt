air_clone/
├── Gemfile
├── main.rb                  # Point d'entrée
├── lib/
│   ├── engine.rb            # Moteur principal
│   ├── game_state.rb        # État global (flags, routes débloquées)
│   ├── scene.rb             # Modèle de scène
│   ├── route.rb             # Gestion des routes
│   └── dsl.rb               # DSL pour écrire les scénarios
├── data/
│   ├── characters.rb        # Définition des personnages
│   ├── dream_route.rb       # Route DREAM (Kano, Misuzu, Minagi)
│   ├── summer_route.rb      # Route SUMMER (Ryuya / Kanna)
│   └── air_route.rb         # Route AIR (true end)
└── README.md