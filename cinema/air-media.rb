# data/air_media.rb
module AirFlight
  # Source: bilibili.com/bangumi/media/md1816
  AIR_MEDIA = {
    title: "AIR",
    original_title: "Air",
    studio: "Kyoto Animation (京都アニメーション)",
    director: "Tatsuya Ishihara (石原立也)",
    series_composition: "Fumihiko Shimo (志茂文彦)",
    character_design: "Tomoe Aratani (荒谷朋恵)",
    music: ["Shinji Orito (折戸伸治)", "Magome Togoshi (戸越まごめ)", "Jun Maeda (麻枝准)"],
    original_work: "Key / Visual Art's",
    broadcast_start: "2005-01-06",
    total_episodes: 12,
    bilibili_rating: 9.8,
    bilibili_views: 24_492_000,
    bilibili_followers: 1_827_000,

    # Episode list from the Bilibili page[reference:0]
    episodes: [
      { num: 1,  title: "微风～breeze～",  title_en: "Breeze" },
      { num: 2,  title: "小镇～town～",    title_en: "Town" },
      { num: 3,  title: "细语～whisper～", title_en: "Whisper" },
      { num: 4,  title: "羽毛～plume～",   title_en: "Plume" },
      { num: 5,  title: "翼～wing～",      title_en: "Wing" },
      { num: 6,  title: "星～star～",      title_en: "Star" },
      { num: 7,  title: "梦～dream～",     title_en: "Dream" },
      { num: 8,  title: "夏～summer～",    title_en: "Summer" },
      { num: 9,  title: "月～moon～",      title_en: "Moon" },
      { num: 10, title: "光～light～",     title_en: "Light" },
      { num: 11, title: "海～sea～",       title_en: "Sea" },
      { num: 12, title: "空～air～",       title_en: "Air" }
    ],

    # Voice cast from the Bilibili page[reference:1]
    cast: {
      "国崎往人" => "小野大輔",
      "神尾観鈴" => "川上とも子",
      "霧島佳乃" => "岡本麻見",
      "遠野美凪" => "柚木涼香",
      "神奈備命" => "西村ちなみ",
      "神尾晴子" => "久川綾",
      "霧島聖"   => "冬馬由美",
      "みちる"   => "田村ゆかり",
      "ポテト"   => "今野宏美"
    },

    # Key songs from the AIR franchise
    soundtrack: [
      { title: "鳥の詩",           artist: "Lia",              type: :opening },
      { title: "青空",             artist: "Lia",              type: :insert },
      { title: "夏影",             artist: "Jun Maeda",        type: :bgm },
      { title: "月童",             artist: "Shinji Orito",     type: :bgm },
      { title: "ふたり",           artist: "Lia",              type: :ending },
      { title: "Farewell song",    artist: "Lia",              type: :ending }
    ],

    # Synopsis from Bilibili[reference:2]
    synopsis: <<~SYNOPSIS
      夏天，在靠近海边小小的街道上，一位青年从公共汽车站下车了。
      青年一直在旅行中，这样的小镇他无心久留。在赚足了路费后，
      他会去那些更繁华的地方。他的旅伴，是母亲留给他的小小的人偶——
      他不用手接触，人偶就能动起来。从母亲那里继承了这种魔法，
      靠着表演人偶维生，他的旅途才一直能继续着……
      就在这条海边的街上，青年和命运中的少女相遇了。
    SYNOPSIS
  }.freeze
end