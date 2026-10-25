<?php
// RSS를 대신 읽어 JSON으로 내보내는 중계 파일 (도톰 호스팅에 올려서 사용)
// 사용: jin_rss.php?url=https://rss.blog.naver.com/아이디.xml   (url 없으면 아래 기본 주소)
header('Content-Type: application/json; charset=utf-8');
header('Access-Control-Allow-Origin: *');

$DEFAULT = 'https://rss.blog.naver.com/fb1234.xml';
// 읽어 올 수 있는 사이트 (이 도메인과 그 하위만 허용) — 필요하면 추가하세요
$ALLOW = ['naver.com', 'tistory.com', 'blogspot.com', 'wordpress.com', 'youtube.com'];

$url = isset($_GET['url']) ? trim($_GET['url']) : $DEFAULT;
$p = @parse_url($url);
$host = (is_array($p) && isset($p['host'])) ? strtolower($p['host']) : '';
$okHost = false;
foreach ($ALLOW as $d) { if ($host === $d || substr($host, -strlen('.' . $d)) === '.' . $d) $okHost = true; }
if (!is_array($p) || !isset($p['scheme']) || $p['scheme'] !== 'https' || isset($p['user']) || (isset($p['port']) && $p['port'] != 443) || !$okHost) {
  http_response_code(400); echo '{"items":[],"error":"not allowed"}'; exit;
}

$cache = __DIR__ . '/jin_rss_' . md5($url) . '.json';
if (is_file($cache) && time() - filemtime($cache) < 180) { readfile($cache); exit; }   // 3분 캐시

$xml = false;
if (function_exists('curl_init')) {
  $ch = curl_init($url);
  curl_setopt_array($ch, [CURLOPT_RETURNTRANSFER => true, CURLOPT_TIMEOUT => 8, CURLOPT_FOLLOWLOCATION => false, CURLOPT_USERAGENT => 'Mozilla/5.0']);
  $xml = curl_exec($ch); curl_close($ch);
}
if (!$xml) {
  $ctx = stream_context_create(['http' => ['timeout' => 8, 'header' => "User-Agent: Mozilla/5.0\r\n"]]);
  $xml = @file_get_contents($url, false, $ctx);
}

$clean = function ($t) { return mb_substr(trim(preg_replace('/\s+/', ' ', strip_tags((string)$t))), 0, 120); };
$items = []; $source = '';
if ($xml) {
  $x = @simplexml_load_string($xml, 'SimpleXMLElement', LIBXML_NOCDATA);
  if ($x && isset($x->channel->item)) {            // RSS 2.0
    $source = (string)$x->channel->title;
    foreach ($x->channel->item as $it) {
      if (count($items) >= 8) break;
      $items[] = ['title' => (string)$it->title, 'link' => (string)$it->link, 'date' => (string)$it->pubDate, 'summary' => $clean($it->description)];
    }
  } elseif ($x && isset($x->entry)) {              // Atom (유튜브 채널 등)
    $source = (string)$x->title;
    foreach ($x->entry as $e) {
      if (count($items) >= 8) break;
      $items[] = ['title' => (string)$e->title, 'link' => (string)$e->link['href'], 'date' => (string)($e->published ?: $e->updated), 'summary' => $clean($e->summary ?: $e->content)];
    }
  }
}
$out = json_encode(['source' => $source, 'items' => $items], JSON_UNESCAPED_UNICODE);
if ($items) @file_put_contents($cache, $out);
echo $out;
