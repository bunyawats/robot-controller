//! A small JSON reader for MQTT payloads.
//!
//! Hand-written rather than serde_json so the module has no dependencies and its tests run on
//! the host with plain `rustc` (see CLAUDE.md). Supports all of JSON except that nesting is
//! limited to `MAX_DEPTH` levels, which is plenty for command payloads.

/// Deepest nesting accepted, so a hostile payload can't overflow the stack.
const MAX_DEPTH: u32 = 8;

#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<Value>),
    /// Keys in the order they appeared.
    Obj(Vec<(String, Value)>),
}

impl Value {
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Num(n) => Some(*n),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::Str(s) => Some(s),
            _ => None,
        }
    }
}

/// Parses one JSON value; anything but whitespace after it is an error.
pub fn parse(text: &str) -> Result<Value, String> {
    let mut p = Parser { s: text.as_bytes(), i: 0 };
    let value = p.value(0)?;
    p.skip_ws();
    if p.i != p.s.len() {
        return Err(format!("unexpected text after the JSON value at byte {}", p.i));
    }
    Ok(value)
}

struct Parser<'a> {
    s: &'a [u8],
    i: usize,
}

impl Parser<'_> {
    fn peek(&self) -> Option<u8> {
        self.s.get(self.i).copied()
    }

    fn skip_ws(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\t' | b'\n' | b'\r')) {
            self.i += 1;
        }
    }

    fn expect(&mut self, c: u8) -> Result<(), String> {
        self.skip_ws();
        if self.peek() == Some(c) {
            self.i += 1;
            Ok(())
        } else {
            Err(format!("expected '{}' at byte {}", c as char, self.i))
        }
    }

    fn value(&mut self, depth: u32) -> Result<Value, String> {
        if depth > MAX_DEPTH {
            return Err("JSON nested too deeply".into());
        }
        self.skip_ws();
        match self.peek() {
            Some(b'{') => self.object(depth),
            Some(b'[') => self.array(depth),
            Some(b'"') => self.string().map(Value::Str),
            Some(b't') => self.word("true", Value::Bool(true)),
            Some(b'f') => self.word("false", Value::Bool(false)),
            Some(b'n') => self.word("null", Value::Null),
            Some(c) if c == b'-' || c.is_ascii_digit() => self.number(),
            Some(_) => Err(format!("unexpected character at byte {}", self.i)),
            None => Err("unexpected end of JSON".into()),
        }
    }

    fn word(&mut self, word: &str, value: Value) -> Result<Value, String> {
        if self.s[self.i..].starts_with(word.as_bytes()) {
            self.i += word.len();
            Ok(value)
        } else {
            Err(format!("unexpected word at byte {}", self.i))
        }
    }

    fn number(&mut self) -> Result<Value, String> {
        let start = self.i;
        while matches!(self.peek(), Some(c) if c.is_ascii_digit() || b"+-.eE".contains(&c)) {
            self.i += 1;
        }
        // Only ASCII bytes were consumed, so this is valid UTF-8.
        let text = std::str::from_utf8(&self.s[start..self.i]).unwrap_or_default();
        match text.parse::<f64>() {
            Ok(n) if n.is_finite() => Ok(Value::Num(n)),
            _ => Err(format!("bad number '{text}'")),
        }
    }

    fn string(&mut self) -> Result<String, String> {
        self.i += 1; // opening quote
        let mut out = Vec::new();
        loop {
            let c = self.peek().ok_or("unterminated string")?;
            self.i += 1;
            match c {
                b'"' => break,
                b'\\' => {
                    let esc = self.peek().ok_or("unterminated string")?;
                    self.i += 1;
                    match esc {
                        b'"' | b'\\' | b'/' => out.push(esc),
                        b'n' => out.push(b'\n'),
                        b't' => out.push(b'\t'),
                        b'r' => out.push(b'\r'),
                        b'b' => out.push(0x08),
                        b'f' => out.push(0x0c),
                        b'u' => {
                            let hex = self
                                .s
                                .get(self.i..self.i + 4)
                                .and_then(|h| std::str::from_utf8(h).ok())
                                .and_then(|h| u32::from_str_radix(h, 16).ok())
                                .ok_or("bad \\u escape")?;
                            self.i += 4;
                            let ch = char::from_u32(hex).ok_or("unsupported \\u escape")?;
                            out.extend_from_slice(ch.encode_utf8(&mut [0; 4]).as_bytes());
                        }
                        _ => return Err(format!("bad escape at byte {}", self.i - 1)),
                    }
                }
                _ => out.push(c),
            }
        }
        String::from_utf8(out).map_err(|_| "string is not valid UTF-8".into())
    }

    fn array(&mut self, depth: u32) -> Result<Value, String> {
        self.i += 1; // [
        let mut items = Vec::new();
        self.skip_ws();
        if self.peek() == Some(b']') {
            self.i += 1;
            return Ok(Value::Arr(items));
        }
        loop {
            items.push(self.value(depth + 1)?);
            self.skip_ws();
            match self.peek() {
                Some(b',') => self.i += 1,
                Some(b']') => {
                    self.i += 1;
                    return Ok(Value::Arr(items));
                }
                _ => return Err(format!("expected ',' or ']' at byte {}", self.i)),
            }
        }
    }

    fn object(&mut self, depth: u32) -> Result<Value, String> {
        self.i += 1; // {
        let mut fields = Vec::new();
        self.skip_ws();
        if self.peek() == Some(b'}') {
            self.i += 1;
            return Ok(Value::Obj(fields));
        }
        loop {
            self.skip_ws();
            if self.peek() != Some(b'"') {
                return Err(format!("expected a key at byte {}", self.i));
            }
            let key = self.string()?;
            self.expect(b':')?;
            fields.push((key, self.value(depth + 1)?));
            self.skip_ws();
            match self.peek() {
                Some(b',') => self.i += 1,
                Some(b'}') => {
                    self.i += 1;
                    return Ok(Value::Obj(fields));
                }
                _ => return Err(format!("expected ',' or '}}' at byte {}", self.i)),
            }
        }
    }
}

/// `text` as a JSON string literal, quotes included.
pub fn quote(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('"');
    for c in text.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_command_object() {
        let v = parse(r#" {"cmd": "walk", "steps": 4, "joints": ["s4", "r_elbow"], "x": [true, null, -1.5e1]} "#)
            .unwrap();
        assert_eq!(
            v,
            Value::Obj(vec![
                ("cmd".into(), Value::Str("walk".into())),
                ("steps".into(), Value::Num(4.0)),
                (
                    "joints".into(),
                    Value::Arr(vec![Value::Str("s4".into()), Value::Str("r_elbow".into())])
                ),
                (
                    "x".into(),
                    Value::Arr(vec![Value::Bool(true), Value::Null, Value::Num(-15.0)])
                ),
            ])
        );
    }

    #[test]
    fn parses_bare_values_and_empty_containers() {
        assert_eq!(parse("120").unwrap(), Value::Num(120.0));
        assert_eq!(parse("{}").unwrap(), Value::Obj(vec![]));
        assert_eq!(parse("[ ]").unwrap(), Value::Arr(vec![]));
    }

    #[test]
    fn handles_string_escapes() {
        assert_eq!(
            parse(r#""a\"b\\c\né""#).unwrap(),
            Value::Str("a\"b\\c\n\u{e9}".into())
        );
    }

    #[test]
    fn rejects_broken_json() {
        for bad in [
            "", "{", "{\"a\"}", "{\"a\":1,}", "[1 2]", "\"open", "{a:1}", "1 2", "--1", "tru",
            "1e999",
        ] {
            assert!(parse(bad).is_err(), "accepted {bad:?}");
        }
    }

    #[test]
    fn rejects_deep_nesting() {
        let deep = "[".repeat(100) + &"]".repeat(100);
        assert!(parse(&deep).is_err());
    }

    #[test]
    fn quote_round_trips() {
        for s in ["plain", "q\"uote", "back\\slash", "line\nbreak", "\u{1}ctl", "é"] {
            assert_eq!(parse(&quote(s)).unwrap(), Value::Str(s.into()));
        }
    }
}
