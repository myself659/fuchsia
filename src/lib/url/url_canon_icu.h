// Copyright 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef SRC_LIB_URL_URL_CANON_ICU_H_
#define SRC_LIB_URL_URL_CANON_ICU_H_

// ICU integration functions.

#include "src/lib/fxl/compiler_specific.h"
#include "src/lib/url/url_canon.h"
#include "src/lib/url/url_canon_internal.h"
#include "src/lib/url/url_export.h"

typedef struct UConverter UConverter;

namespace url {

// An implementation of CharsetConverter that implementations can use to
// interface the canonicalizer with ICU's conversion routines.
class URL_EXPORT ICUCharsetConverter : public CharsetConverter {
 public:
  // Constructs a converter using an already-existing ICU character set
  // converter. This converter is NOT owned by this object; the lifetime must
  // be managed by the creator such that it is alive as long as this is.
  ICUCharsetConverter(UConverter* converter);

  ~ICUCharsetConverter() override;

  void ConvertFromUTF16(const uint16_t* input, size_t input_len,
                        CanonOutput* output) override;

 private:
  // The ICU converter, not owned by this class.
  UConverter* converter_;
};

}  // namespace url

#endif  // SRC_LIB_URL_URL_CANON_ICU_H_
