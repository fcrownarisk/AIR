# data/dream_route.rb
module AirClone
  DREAM_ROUTE = DSL.new.tap do |dsl|

    dsl.scene("dream_opening") do
      background "Ville côtière, été"
      bgm "Natsukage"

      narrate "Le bus s'arrête dans une petite ville au bord de la mer."
      narrate "Un jeune homme descend, un pantin de bois à la main."
      say :yukito, "Encore une ville... J'espère au moins gagner de quoi manger."
      say :yukito, "La 'fille du ciel'... Où peut-elle bien être ?"
      next_scene "dream_meet_misuzu"
    end

    dsl.scene("dream_meet_misuzu") do
      background "Rue principale"
      bgm "Natsukage"

      narrate "Une jeune fille aux longs cheveux blonds s'approche."
      say :misuzu, "Bonjour ! Tu es nouveau, non ? Tu veux venir chez moi ?"
      say :yukito, "... Quoi ? On vient juste de se rencontrer."
      say :misuzu, "Gao~ ! Tu as faim, non ? Ma mère a fait un gâteau !"
      say :yukito, "... Bon. C'est d'accord."

      choice "La suivre chez elle", goto: "dream_misuzu_home", flag: :met_misuzu, value: true
      choice "Refuser poliment",     goto: "dream_wander_alone"
    end

    dsl.scene("dream_misuzu_home") do
      background "Maison Kamio"
      bgm "Natsukage"

      narrate "La maison de Misuzu est simple, mais chaleureuse."
      say :misuzu, "Maman ! J'ai ramené un invité !"
      say :yukito, "Attends, je n'ai jamais dit que..."

      narrate "Une femme arrive en trombe dans la pièce."
      # ... (Haruko fait son entrée)
      next_scene "dream_choice_route"
    end

    dsl.scene("dream_choice_route") do
      background "Chambre de Misuzu"

      narrate "Les jours passent. Yukito doit choisir quelle fille aider."
      choice "Aider Misuzu",   goto: "misuzu_route_start", flag: :chose_misuzu, value: true
      choice "Aider Kano",     goto: "kano_route_start",   flag: :chose_kano,   value: true
      choice "Aider Minagi",   goto: "minagi_route_start", flag: :chose_minagi, value: true
    end

    dsl.scene("dream_wander_alone") do
      narrate "Yukito erre dans les rues. Il finit par rencontrer une autre fille."
      next_scene "dream_choice_route"
    end

    # ── Route Misuzu ──
    dsl.scene("misuzu_route_start") do
      background "Plage au coucher du soleil"
      bgm "Aozora"

      narrate "Misuzu regarde le ciel, les bras tendus."
      say :misuzu, "Un jour... je volerai. J'en suis sûre."
      say :yukito, "... Pourquoi es-tu si sûre ?"
      say :misuzu, "Parce que je le vois dans mes rêves. Un ciel infini."

      choice "Lui prendre la main", goto: "misuzu_good_end", flag: :misuzu_affection, value: true
      choice "Rester silencieux",   goto: "misuzu_normal_end"
    end

    dsl.scene("misuzu_good_end") do
      background "Ciel étoilé"
      bgm "Aozora"

      narrate "Le destin de Misuzu commence à changer."
      say :yukito, "Je resterai avec toi. Jusqu'à ce que tu puisses voler."
      say :misuzu, "... Merci, Yukito."
      narrate "Fin de la route de Misuzu. ★"
      # signal de fin de route pour le GameState
      narrate "__ROUTE_COMPLETE__:misuzu"
      next_scene nil
    end

    dsl.scene("misuzu_normal_end") do
      narrate "Le silence s'installe. Un lien s'est peut-être perdu."
      next_scene nil
    end

    # ... (Routes Kano et Minagi similaires)
  end
end