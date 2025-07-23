// Custom commitlint configuration with English-only rules using commitlint-plugin-function-rules

module.exports = {
  extends: ['@commitlint/config-conventional'],
  plugins: ['commitlint-plugin-function-rules'],

  rules: {
    // Type rules
    'type-enum': [
      2,
      'always',
      ['feat', 'fix', 'docs', 'style', 'refactor', 'perf', 'test', 'chore', 'ci', 'build', 'revert'],
    ],
    'type-case': [2, 'always', 'lower-case'],
    'type-empty': [2, 'never'],

    // Scope rules
    'scope-case': [2, 'always', 'lower-case'],

    // Subject rules
    'subject-empty': [2, 'never'],
    'subject-full-stop': [2, 'never', '.'],

    // English-only validation using function rules (override existing rules)
    'subject-case': [0], // Disable original rule
    'function-rules/subject-case': [
      2,
      'always',
      (parsed) => {
        const { subject } = parsed;
        if (!subject) return [true];

        // Check for Chinese characters (CJK Unified Ideographs)
        const chineseRegex = /[\u4e00-\u9fff]/;
        if (chineseRegex.test(subject)) {
          return [false, 'Subject must be in English only. Chinese characters are not allowed.'];
        }

        // Also check original case rules
        if (/^[A-Z]/.test(subject)) {
          return [false, 'Subject must not start with uppercase'];
        }

        return [true];
      },
    ],

    // Header rules
    'header-max-length': [2, 'always', 72],

    // Body rules
    'body-empty': [1, 'never'],
    'body-leading-blank': [2, 'always'],
    'body-max-line-length': [2, 'always', 100],

    'body-case': [0], // Disable original rule
    'function-rules/body-case': [
      2,
      'always',
      (parsed) => {
        const { body } = parsed;
        if (!body) return [true];

        // Check for Chinese characters (CJK Unified Ideographs)
        const chineseRegex = /[\u4e00-\u9fff]/;
        if (chineseRegex.test(body)) {
          return [false, 'Body must be in English only. Chinese characters are not allowed.'];
        }

        return [true];
      },
    ],

    // Footer rules
    'footer-leading-blank': [1, 'always'],
    'footer-max-line-length': [2, 'always', 100],
  },
};
