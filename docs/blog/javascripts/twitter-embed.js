(() => {
  const STATUS_URL_RE = /^https?:\/\/(?:twitter|x)\.com\/[^/]+\/status\/(\d+)/i;

  function normalizeStatusUrl(url) {
    try {
      const u = new URL(url);
      const match = `${u.origin}${u.pathname}`.match(STATUS_URL_RE);
      if (!match) return null;
      return `https://twitter.com/i/web/status/${match[1]}`;
    } catch {
      return null;
    }
  }

  function convertBlockquotesToTweetEmbeds() {
    let converted = 0;
    const blockquotes = document.querySelectorAll("blockquote");

    blockquotes.forEach((blockquote) => {
      if (blockquote.classList.contains("twitter-tweet")) return;

      const links = blockquote.querySelectorAll("a[href]");
      let statusUrl = null;

      for (const link of links) {
        const normalized = normalizeStatusUrl(link.href);
        if (normalized) {
          statusUrl = normalized;
          break;
        }
      }

      if (!statusUrl) return;

      const embed = document.createElement("blockquote");
      embed.className = "twitter-tweet";
      embed.setAttribute("data-conversation", "none");

      const anchor = document.createElement("a");
      anchor.href = statusUrl;
      embed.appendChild(anchor);

      blockquote.replaceWith(embed);
      converted += 1;
    });

    return converted;
  }

  function ensureTwitterWidgets() {
    if (window.twttr?.widgets) {
      window.twttr.widgets.load();
      return;
    }

    if (document.querySelector('script[data-twitter-widgets="1"]')) {
      return;
    }

    const script = document.createElement("script");
    script.src = "https://platform.twitter.com/widgets.js";
    script.async = true;
    script.charset = "utf-8";
    script.setAttribute("data-twitter-widgets", "1");
    document.head.appendChild(script);
  }

  function init() {
    const converted = convertBlockquotesToTweetEmbeds();
    if (converted > 0) {
      ensureTwitterWidgets();
    }
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init, { once: true });
  } else {
    init();
  }
})();
