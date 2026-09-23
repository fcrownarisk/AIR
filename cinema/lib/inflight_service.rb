# lib/inflight_service.rb
module AirFlight
  class InFlightService
    MENU = {
      drinks: ["緑茶 (Green Tea)", "オレンジジュース", "コーヒー", "ミネラルウォーター"],
      snacks: ["せんべい", "Pocky", "和菓子", "チョコレート"],
      meals:  ["幕の内弁当", "海鮮丼", "カレーライス"]
    }.freeze

    def initialize(theater)
      @theater = theater
      @orders = []
    end

    def serve_drinks
      puts "\n🍵 In-flight drink service:"
      @theater.passengers.each do |p|
        drink = MENU[:drinks].sample
        @orders << { passenger: p[:name], item: drink, type: :drink }
        puts "   → #{p[:name]}: #{drink}"
      end
    end

    def serve_meals
      puts "\n🍱 Meal service:"
      @theater.passengers.each do |p|
        meal = MENU[:meals].sample
        @orders << { passenger: p[:name], item: meal, type: :meal }
        puts "   → #{p[:name]}: #{meal}"
      end
    end

    def order_summary
      puts "\n📝 Service Summary (#{@orders.size} items served)"
      @orders.group_by { |o| o[:type] }.each do |type, items|
        puts "  #{type.to_s.capitalize}: #{items.size}"
      end
    end
  end
end